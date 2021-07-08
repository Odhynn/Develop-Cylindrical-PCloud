# -*- coding: utf-8 -*-

import numpy as np
import copy
import time
from PIL import Image
from scipy.interpolate import griddata

import open3d as o3d

import devcyl_vis

print("\nDEVELOPMENT OF CYLINDRICAL SURFACES IN POINT CLOUD FORMAT")
print("  diploma thesis of Odysseus Galanakis")
print("  School of Rural, Surveying and Geoinformatics Engineering")
print("  National Technical University of Athens, 2021")
print()
print("  Utilizing Python 3.8")
print("  with Open3d 0.13: A Modern Library for 3D Data Processing")
print("               by Qian-Yi Zhou, Jaesik Park, Vladlen Koltun")
print()
print("Second Step : Unwrapping the Cylinder")
print()

# set default open3d visualization window size
vww = 1600
vwh = 900

# display arrays in precision 4 decimals
np.set_printoptions(precision=4, suppress=True)


# %% Load

pcd_path = input("Enter point cloud file path: ")
log_path = input("Enter fitting log file path: ")

print(f"\nLoading '{pcd_path}' point cloud...")
tic = time.perf_counter()

pcd = o3d.io.read_point_cloud(pcd_path)
toc = time.perf_counter()
print(f"Loaded in {toc - tic:0.4f} sec.\n")
print(pcd_path, ":", pcd)

poi = np.asarray(pcd.points)
col = np.asarray(pcd.colors)


# %% Projection to Cylinder

# read fitted radius from log file
with open(log_path, "r") as log:
    list_of_lines = list(log)
    r = float(list_of_lines[2])
print("Fitted Cylinder Radius =", r)

# "p" for "projected"
ppcd = copy.deepcopy(pcd)
ppoi = np.asarray(ppcd.points)


print(f"\nProjecting '{pcd_path}' point cloud to cylinder surface...")
tic = time.perf_counter()

# polar into cartesian coordinates with quadrant-aware arctan function
for i in range(len(poi)):
    x = poi[i, 0]
    y = poi[i, 1]
    ppoi[i, 0] = r * np.cos(np.arctan2(y, x))
    ppoi[i, 1] = r * np.sin(np.arctan2(y, x))

toc = time.perf_counter()
print(f"Projected in {toc - tic:0.4f} sec.\n")


# %% Visualization

# draw axes with 1m extra length beyond coordinate bounds
xamin = pcd.get_min_bound()[0] - 1
xamax = pcd.get_max_bound()[0] + 1
yamin = pcd.get_min_bound()[1] - 1
yamax = pcd.get_max_bound()[1] + 1
zamin = pcd.get_min_bound()[2] - 1
zamax = pcd.get_max_bound()[2] + 1

# cylinder centre lies at mean point height
zavg = (zamax + zamin) / 2

x_axis = devcyl_vis.create_line(
    (xamin, 0, zavg), (xamax, 0, zavg), (1, 0, 0))
y_axis = devcyl_vis.create_line(
    (0, yamin, zavg), (0, yamax, zavg), (0, 1, 0))
z_axis = devcyl_vis.create_line((0, 0, zamin), (0, 0, zamax), (0, 0, 1))

# draw axis unit vectors
ax_note = o3d.geometry.TriangleMesh.create_coordinate_frame()
ax_note.translate((0, 0, zavg), relative=False)
zavg_decimals = '{0:.3f}'.format(zavg)
print(f"Visualizing: Unit vector indicator point at (0, 0, {zavg_decimals})")

o3d.visualization.draw_geometries([ppcd, x_axis, y_axis, z_axis, ax_note],
                                  "Projected to Cylinder Surface", vww, vwh)


# %% Leftmost point alignment

# "r" for "rotated"
rpcd = copy.deepcopy(ppcd)
print("\nDetecting leftmost point...")

# the cloud will turn clockwise around z-axis by this increment
pie_slice = 12
yaw_incr = 2*np.pi / pie_slice

# until a gap is detected : no point of the cloud lies on the negative x-axis
R = ppcd.get_rotation_matrix_from_xyz((0, 0, -yaw_incr))
xmin = rpcd.get_min_bound()[0]
incr_rotations = 0

for i in range(pie_slice):
    rpcd.rotate(R, center=(0, 0, 0))
    xmin = rpcd.get_min_bound()[0]
    if abs(abs(xmin)-r) <= 0.000000001:
        incr_rotations += 1
        continue
    else:
        z_searching_rot = incr_rotations * (- yaw_incr)
        break


rpoi = np.asarray(rpcd.points)

if incr_rotations == pie_slice:  # came full circle, did not find a gap
    print("\nLeftmost point not detected.\
          \nDevelopment width will span 360 degrees.")
    align_rot = 0  # no rotation

# find polar angle theta and use it to rotate the leftmost point onto -x axis
else:
    thetamin = np.arccos(xmin / r)  # because now y>0 for the xmin point
    R = ppcd.get_rotation_matrix_from_xyz((0, 0, (np.pi-thetamin)))
    rpcd.rotate(R, center=(0, 0, 0))

    # total angle by which cloud was rotated
    align_rot = z_searching_rot + (np.pi - thetamin)
    print(f"Leftmost point detected after turning {align_rot} rad.")

    rpoi_minindex = np.argmin(rpoi, axis=0)
    print("  Point index: ", rpoi_minindex[0])
    xmin_ppoi = np.asarray(ppcd.points)[rpoi_minindex[0]]
    print("  Before rotation: ", xmin_ppoi)
    xmin_rpoi = np.asarray(rpcd.points)[rpoi_minindex[0]]
    print("  After rotation: ", xmin_rpoi)

    # check visually
    xmin_poi_line = devcyl_vis.create_line(
        xmin_rpoi, (0, 0, xmin_rpoi[2]), (1, 0, 1))
    o3d.visualization.draw_geometries(
        [rpcd, x_axis, y_axis, z_axis, ax_note, xmin_poi_line],
        "Aligned : leftmost point (purple) must lie on the XZ plane", vww, vwh)


# %% Conversion to Polar

def nega_theta(x, y):
    # conventional : counter-clockwise from positive x-axis, range (-pi,pi]
    theta = np.arctan2(y, x)
    # its mirror, but better : clockwise from negative x-axis, range [0, 2*pi)
    nega_theta = - theta + np.pi
    return(nega_theta)


# get a table with the polar coords of the cloud points
print("\nCalculating polar coordinates...")
tic = time.perf_counter()

rpoi_polar = np.empty((len(rpoi), 2), float)
for i in range(len(rpoi)):
    rho = r
    theta = nega_theta(rpoi[i][0], rpoi[i][1])
    rpoi_polar[i] = [rho, theta]

toc = time.perf_counter()
print(f"Calculated in {toc - tic:0.4f} sec.\n")

print("Cartesian coords: ", rpoi)
print("Polar coords: ", rpoi_polar)

thetamin = np.amin(rpoi_polar, axis=0)[1]
thetamax = np.amax(rpoi_polar, axis=0)[1]
print("Minimum theta: ", '{0:.4f}'.format(thetamin))
print("Maximum theta: ", '{0:.4f}'.format(thetamax))


# %% Pixel Size in Object Space

# ask for GSD (real distance between pixel centers)
gsd = input(
    "Define Ground Sampling Distance (m): ")


# real dimensions of developed image (in metres)
devh = rpcd.get_max_bound()[2] - rpcd.get_min_bound()[2]
devw = (thetamax - thetamin) * r

# developed image resolution (in pixels)
Nx = round(devw / gsd)
My = round(devh / gsd)

# empty table for the image -- M rows, N columns, each cell to hold [R,G,B]
devimg = np.zeros((My, Nx, 3), dtype=np.uint8)


# developed image pixel coordinates -- M rows, N columns, each cell holds [x,y]
devxy = np.zeros((My, Nx, 2))

# table ij to pixel centre xy
for i in range(My):
    for j in range(Nx):
        xij = (gsd/2) + j*gsd
        yij = (gsd/2) + My*gsd - (i+1)*gsd
        devxy[i][j] = [xij, yij]

print("\nDeveloped image pixel coordinates (x, y):\n", devxy)


# object space cart. coordinates -- M rows, N columns, each cell holds [X,Y,Z]
devXYZ = np.zeros((My, Nx, 3))
zmin = rpcd.get_min_bound()[2]

for i in range(My):
    for j in range(Nx):
        xij = devxy[i][j][0]
        yij = devxy[i][j][1]

        thetaij = -(xij/r) + np.pi  # conventional counter-clockwise theta
        Xij = np.cos(thetaij) * r
        Yij = np.sin(thetaij) * r
        Zij = yij + zmin
        devXYZ[i][j] = [Xij, Yij, Zij]

print("\nCorresponding pixel coordinates in object space (X, Y, Z):\n", devXYZ)


# %% Linear Transformation to 2D

# "d" for "developed"
dpcd = copy.deepcopy(rpcd)
dpoi = np.asarray(dpcd.points)

print("\nDeveloping by direct linear transformation...")
tic = time.perf_counter()

for i in range(len(rpoi)):
    xd = rpoi_polar[i][1] * r
    yd = rpoi[i][2] - zmin
    dpoi[i] = [xd, yd, 0]

toc = time.perf_counter()
print(f"Unwrapped in {toc - tic:0.4f} sec.")


ax_origin = o3d.geometry.TriangleMesh.create_coordinate_frame()
o3d.visualization.draw_geometries(
    [dpcd, ax_origin], "Unwrapped on XY plane", vww, vwh)


# %% Colour Interpolation

dcol = np.asarray(dpcd.colors)
timestamp = time.strftime("%Y%m%d-%H%M%S")

while True:
    nn_or_2l = input("Select method to resample developed image colour by:\n\
                     enter (1) for Nearest Neighbour\n\
                     enter (2) for Bilinear Interpolation\n\
                     or enter (3) to log and exit: ")

    if nn_or_2l == "1":
        print("Drawing by nearest neighbour...")
        tic = time.perf_counter()

        interpol = griddata(dpoi[:, :2], dcol, devxy, method='nearest')

        img = Image.fromarray(np.uint8(interpol*255), 'RGB')
        img.show()

        toc = time.perf_counter()
        print(f"Drawn in {toc - tic:0.4f} sec.\n")

        img.save(f'Dev-NN-{timestamp}.png')
        print(f"Saved as 'Dev-NN-{timestamp}.png'")
        continue

    elif nn_or_2l == "2":
        print("Drawing with bilinear interpolation...")
        tic = time.perf_counter()

        interpol = griddata(dpoi[:, :2], dcol, devxy,
                            method='linear', fill_value=0)

        img = Image.fromarray(np.uint8(interpol*255), 'RGB')
        img.show()

        toc = time.perf_counter()
        print(f"Drawn in {toc - tic:0.4f} sec.\n")

        img.save(f'Dev-Bilinear-{timestamp}.png')
        print(f"Saved as 'Dev-Bilinear-{timestamp}.png'")
        continue

    elif nn_or_2l == "3":
        break
    else:
        print("Invalid user input.\n")


# %% Logging

# save devXYZ for any further evaluation
np.save(f"devXYZ-{timestamp}", devXYZ)
print("Saved corresponding pixel coordinates in devXYZ-{timestamp}.npy")

# log unwrapping variables
with open(f"Log-Unwrap-{timestamp}.txt", "w") as log:
    log.write("Source point cloud : " + pcd_path + "\n")
    log.write("Source fitting log file : " + log_path + "\n")
    log.write("Leftmost point alignment rotation angle (rad):\n" + align_rot)

    log.write("\nGround Sampling Distance (m):\n" + gsd)
    log.write("\nDeveloped Area Width (m):\n" + devw)
    log.write("\nDeveloped Area Height (m):\n" + devh)
    log.write("\nDeveloped Image Width (pixels):\n" + Nx)
    log.write("\nDeveloped Image Height (pixels):\n" + My)

input("Enter any key to exit.")
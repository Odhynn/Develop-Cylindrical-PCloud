# -*- coding: utf-8 -*-

import numpy as np
import time
import os
import copy

import open3d as o3d

import devcyl_fit_thr
import devcyl_vis

print("\nDEVELOPMENT OF CYLINDRICAL SURFACES IN POINT CLOUD FORMAT")
print("  diploma thesis of Odysseus Galanakis")
print("  School of Rural, Surveying and Geoinformatics Engineering")
print("  National Technical University of Athens, 2021")
print()
print("  Utilizing Python 3.8 with NumPy, SciPy, and Open3D 0.13")
print()
print("First Step : Fitting the Cylinder")
print()

# display arrays in precision 4 decimals
np.set_printoptions(precision=4, suppress=True)

# set default open3d visualization window size
vww = 1600
vwh = 900


# %% Load Cloud

print("Open3d Version: ", o3d.__version__)

# display the current working directory
cwd = os.getcwd()
print(f"\nCurrent working directory: {cwd}")
print("Files in the above folder may be simply specified by name.")

pcd_path = None

# ask the user to specify point cloud file
while True:
    pcd_path = os.path.normpath(input("\nEnter point cloud file path: "))
    if not os.path.isfile(pcd_path):
        print("Invalid path.")
    else:
        break

# load point cloud
print(f"\nLoading '{pcd_path}' point cloud...")
tic = time.perf_counter()

pcd = o3d.io.read_point_cloud(pcd_path)
toc = time.perf_counter()
print(f"Loaded in {toc - tic:0.4f} sec.\n")
print(pcd_path, ":", pcd)


# %% Fitting & Threshold Loop

# loop logging
parameter_log = []
thresh_log = []

# fitting and thresholding loop
while True:

    # least squares fitting and axis-to-points distance calculation
    fitcyl = devcyl_fit_thr.fit_and_calc(pcd)
    parameter_log.append(fitcyl[1][:-1])

    # threshold
    thresh_q = input("\nApply distance threshold to crop off outliers?"
                     "\nEnter (y) to crop, any other key to continue: ")
    if thresh_q == "y":
        pcd1, thresh = devcyl_fit_thr.thresh(
            fitcyl[0], fitcyl[1], fitcyl[2], fitcyl[3])
        print("\nRepeating cylinder fitting on the new cloud...")
        pcd = pcd1
        thresh_log.append(thresh)
    else:
        break


pcd = fitcyl[0]
cyl_p = fitcyl[1]
mesh = devcyl_vis.cyl_mesh(pcd, cyl_p)

xk = fitcyl[1][0]
yk = fitcyl[1][1]
rot_x = fitcyl[1][2]
rot_y = fitcyl[1][3]
cyl_radius = fitcyl[1][4]
cyl_axis = fitcyl[3]


# %% Alignment to Z Axis

# transform cloud and fitted cylinder to align with z-axis
R = pcd.get_rotation_matrix_from_xyz((-rot_x, -rot_y, 0))
pcd_z = copy.deepcopy(pcd).rotate(R, center=(xk, yk, 0))
mesh_z = copy.deepcopy(mesh).rotate(R, center=(xk, yk, 0))
pcd_z.translate((-xk, -yk, 0))
mesh_z.translate((-xk, -yk, 0))

# visualize purple z_axis to check alignment
zamin = pcd_z.get_min_bound()[2] - 1
zamax = pcd_z.get_max_bound()[2] + 1
z_axis = devcyl_vis.create_line((0, 0, zamin), (0, 0, zamax), (.7, 0, .7))

o3d.visualization.draw_geometries(
    [pcd_z, mesh_z, z_axis], 'Final (Z-axis is purple)', vww, vwh)

# save aligned cloud
timestamp = time.strftime("%Y%m%d-%H%M%S")
pcd_name = f"PointCloud-{timestamp}.ply"
o3d.io.write_point_cloud(pcd_name, pcd_z)
print(f"Aligned cloud saved as {pcd_name}.")


# %% Logging

# save fitting log, with cylinder radius on 3rd line

with open(f"Log-Fit-{timestamp}.txt", "w") as log:
    log.write("Source point cloud : " + pcd_path + "\n")
    log.write("Fitted Cylinder Radius (m):\n" + str(cyl_radius))

    log.write("\n\n---------------------------\nFitted cylinder parameters:\n")
    log.write("xk (m): \n" + str(parameter_log[-1][0]) + "\n")
    log.write("yk (m): \n" + str(parameter_log[-1][1]) + "\n")
    log.write("roll (rad): \n" + str(parameter_log[-1][2]) + "\n")
    log.write("pitch (rad): \n" + str(parameter_log[-1][3]) + "\n")

    log.write("\n---------------------------\nFitting loop iterations:\n")
    for i in range(len(parameter_log)):
        log.write("\nIteration " + str(i+1) + "\n")
        log.write("xk : " + "{:.3f}".format(parameter_log[i][0]) + "\n")
        log.write("yk : " + "{:.3f}".format(parameter_log[i][1]) + "\n")
        log.write("roll : " + "{:.3f}".format(parameter_log[i][2]) + "\n")
        log.write("pitch : " + "{:.3f}".format(parameter_log[i][3]) + "\n")
        log.write("radius : " + "{:.3f}".format(parameter_log[i][4]) + "\n")
        i += 1
    
    log.write("\n-------------------\nThresholds applied:\n\n")
    for i in range(len(thresh_log)):
        log.write("iteration " + str(i+1) + " : " + str(thresh_log[i]) + "m\n")
        i += 1

print(f"Log file saved as Log-Fit-{timestamp}.txt")

print("\nFURTHER STEPS:")
print("Execute 'devcyl_2_unwrap.py' and provide it with the above saved")
print(".ply file and .txt log to develop it to a 2D image. ")
print("In order to only develop a specific area within the point cloud")
print("execute 'devcyl_crop.py' before that and use its product .ply instead.")

input("Press Enter to exit.")


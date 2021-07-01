# -*- coding: utf-8 -*-

# %% Import & Load

import open3d as o3d
import numpy as np
import copy
import time
import visualize_cyl
from scipy.optimize import leastsq
import matplotlib.pyplot as plt

# set default open3d visualization window size
vw_width = 1600
vw_height = 900

# display arrays in precision 4 decimals
np.set_printoptions(precision=4, suppress=True)


# %% Least Squares Fitting

def v_cyl_leastsq(xyz, p):
    """
    xyz = matrix of at least 5 rows
                         and 3 columns (x,y,z of a cylindrical surface)
    p = 5-element array, initial values of the parameter
        p[0] = Xk, x coordinate of the cylinder centre (m)
        p[1] = Yk, y coordinate of the cylinder centre (m)
        p[2] = omega, rotation angle about the x-axis (rad)
        p[3] = phi, rotation angle about the y-axis (rad)
        [4] = r, radius of the cylinder (m)
    ------ RETURNS ------------
    est_p = 5-element array, least squares estimate of p
    """

    x = xyz[:, 0]
    y = xyz[:, 1]
    z = xyz[:, 2]

    def rotation_func(p, x, y, z):
        return ((np.cos(p[3])*(p[0] - x) + np.cos(p[2])*np.sin(p[3])*z
                + np.sin(p[2])*np.sin(p[3])*(p[1] - y))**2
                + (np.cos(p[2])*(p[1] - y) - np.sin(p[2])*z)**2)

    def error_func(p, x, y, z):
        return rotation_func(p, x, y, z) - p[4]**2

    est_p, cov_p = leastsq(error_func, p, args=(x, y, z))

    return est_p


def cylinder_fitting(xyz, p):
    """
    Applies least squares estimate of the functions above
    to fit a cylinder to scattered points in space
    --- RETURNS ------------
    cyl_p = 6-element list, estimated cylinder parametres
        cyl_p[0] = xk, x coordinate of the cylinder centre (m)
        cyl_p[1] = xk, y coordinate of the cylinder centre (m)
        cyl_p[2] = omega, rotation angle about the x-axis (rad)
        cyl_p[3] = phi, rotation angle about the y-axis (rad)
        cyl_p[4] = r, radius of the cylinder (m)
        cyl_p[5] = RyRx, auxiliary rotation matrix (3x3 numpy array)
    """

    print("\nFitting cylinder... ")
    tic = time.perf_counter()

    est_p = v_cyl_leastsq(xyz, p)

    toc = time.perf_counter()
    print(f"Fitted in {toc - tic:0.4f} sec.\n")

    # print parametres
    xk = est_p[0]
    yk = est_p[1]
    rot_x = est_p[2]  # omega = roll
    rot_y = est_p[3]  # phi = pitch
    cyl_radius = abs(est_p[4])
    print(f"Estimated parameters: \nxk = {xk} m\nyk = {yk} m"
          f"\nroll = {rot_x} rad\npitch = {rot_y} rad"
          f"\ncylinder radius = {cyl_radius} m")

    cosw = np.cos(rot_x)
    sinw = np.sin(rot_x)
    cosf = np.cos(rot_y)
    sinf = np.sin(rot_y)

    # calculate RyRx rotation matrix and print it
    rot_matrix = np.array(((cosf, sinw*sinf, sinf*cosw),
                           (0, cosw, -sinw),
                           (-sinf, cosf*sinw, cosf*cosw)))
    print("\nCylinder Rotation Matrix:\n", rot_matrix)

    cyl_p = [xk, yk, rot_x, rot_y, cyl_radius, rot_matrix]
    return cyl_p


# %% Fitting

def fit_and_calc(pcd):
    """
    Loads a point cloud and uses the above functions to fit a vertical cylinder
    to it and calculates how close each point comes to the cylinder surface.
    Returns a tuple of:
        the same point cloud,
        an array containing the 5 fitting parameters (xk, yk, roll, pitch, r)
        an array containing all the point-to-surface distances
        a segment of the cylinder axis
    """

    # Read point cloud from PLY
    pcd_pts = np.asarray(pcd.points)
    print(np.asarray(pcd.points), "\n")

    print('Maximum xyz:', pcd.get_max_bound())
    print('Minimum xyz:', pcd.get_min_bound())

    # %% Fitting

    # ask the user to provide initial values
    init_p = np.array([1000, 1000, 0, 0, 1])

    print("\nDefine Initial Parameters:")
    init_p[0] = input("Xk, x coordinate of the cylinder centre (m) = ")
    init_p[1] = input("Yk, y coordinate of the cylinder centre (m) = ")
    init_p[2] = input("omega, rotation angle about the x-axis (rad) = ")
    init_p[3] = input("phi, rotation angle about the y-axis (rad) = ")
    init_p[4] = input("r, radius of the cylinder (m) = ")
    print("\nInitial Parameters: ", init_p)

    # run the fitting script
    cyl_p = cylinder_fitting(pcd_pts, init_p)

    # %% Visualize Mesh

    axis_max = round(pcd.get_min_bound()[2])-1
    axis_min = round(pcd.get_max_bound()[2])+1

    cyl_axis = visualize_cyl.rotate_line(
        cyl_p[0], cyl_p[1], axis_min, axis_max, cyl_p[5])

    # visualize fitted cylinder
    wireframe = visualize_cyl.cyl_mesh(pcd, cyl_p)
    o3d.visualization.draw_geometries(
        [pcd, wireframe, cyl_axis], 'Fitted Cylinder', vw_width, vw_height)

    # %% Distance Calc

    def p2l_distance(a, b, p):
        """
        Point to Line Distance, for any dimension N (Vector Formulation)
        ----------
        a, b: any two points on the line (np.array[N,1])
        p : arbitrary point to which the distance is measured (np.array[N,1])
        """
        n = (a-b)/np.linalg.norm(a-b)
        return np.linalg.norm((p-a)-np.dot(p-a, n)*n)

    # get two points on the cylinder axis
    cyl_axis_pts = o3d.geometry.LineSet.get_line_coordinate(cyl_axis, 0)

    print("\nCalculating point distances from cylinder axis,")
    print("(may take several minutes)...")

    # create empty N*1 array to point distances from cylinder axis
    cyl_axis_distances = np.zeros((np.shape(pcd_pts)[0], 1))
    tic = time.perf_counter()

    for i in range(np.shape(pcd_pts)[0]):
        cyl_axis_distances[i] = p2l_distance(
            cyl_axis_pts[0], cyl_axis_pts[1], pcd_pts[i])
    print(cyl_axis_distances)

    toc = time.perf_counter()
    print(f"Completed in {toc - tic:0.4f} sec.")

    print("\nCalculating point distances from cylinder radius...")
    cyl_s2p_dist = abs(cyl_axis_distances - cyl_p[4])
    print(cyl_s2p_dist)
    print("\nMax. point distance from surface:\n", np.max(cyl_s2p_dist), "m")
    plt.hist(cyl_s2p_dist, bins=100)
    plt.xlabel("Distance (m)")
    plt.ylabel("Number of points")
    plt.title("Point distances from fitted cylinder")
    plt.show()

    return pcd, cyl_p, cyl_s2p_dist, cyl_axis


# %% Threshold

def thresh(pcd, cyl_p, cyl_s2p_dist, cyl_axis):
    """
    Receives the output of the above fit-and-calc function
    and removes any point in the cloud farther than a given threshold distance.
    A preview of the cropping is given and the user decides
    whether to try a different threshold value or proceed.
    """

    while True:
        # request the threshold for admitting points into the next iteration
        th = float(input("Selection Threshold"
                         "\nDefine max. point distance to cyl. surface (m): "))
        # but not larger than the cylinder radius
        if th >= cyl_p[4]:
            print("Selection Threshold must be less than the cylinder radius!")
            continue

        # create new point cloud according to threshold condition
        pcd1 = pcd.select_by_index(
            np.where(cyl_s2p_dist[:, 0] <= th)[0])

        # create two cylinders to visually convey the threshold boundaries
        cyl_p_th1 = copy.deepcopy(cyl_p)
        cyl_p_th2 = copy.deepcopy(cyl_p)
        cyl_p_th1[4] = cyl_p[4] + th
        cyl_p_th2[4] = cyl_p[4] - th

        thresh1 = visualize_cyl.cyl_mesh(pcd, cyl_p_th1, 3, True)
        thresh2 = visualize_cyl.cyl_mesh(pcd, cyl_p_th2, 3, True)

        # two consecutive visualizations convey effect of threshold cropping
        o3d.visualization.draw_geometries([pcd, thresh1, thresh2, cyl_axis],
                                          'Original cloud and cropping guides',
                                          vw_width, vw_height)
        o3d.visualization.draw_geometries([pcd1, cyl_axis],
                                          'Cropped point cloud preview',
                                          vw_width, vw_height)

        # "continue or redefine?"
        thr_chk = input("\nRecalibrate cylinder on cropped cloud? (y/n)"
                        "\nEnter (y) to continue, (n) to redefine threshold: ")
        if thr_chk == "y":
            break
        elif thr_chk == "n":
            continue
        else:
            print("Invalid user input.")

    return pcd1, th

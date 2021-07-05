# -*- coding: utf-8 -*-

import copy
import time

import open3d as o3d

print("\nDEVELOPMENT OF CYLINDRICAL SURFACES IN POINT CLOUD FORMAT")
print("  diploma thesis of Odysseus Galanakis")
print("  School of Rural, Surveying and Geoinformatics Engineering")
print("  National Technical University of Athens, 2021")
print()
print("  Utilizing Python 3.8")
print("  with Open3d 0.13: A Modern Library for 3D Data Processing")
print("               by Qian-Yi Zhou, Jaesik Park, Vladlen Koltun")
print()
print("Intermediate Step : Cropping to Desired Area")
print()

# set default open3d visualization window size
vww = 1600
vwh = 900


# %% Development Area Selection

def crop_pcd_area(pcd):
    """
    User can crop given point cloud to an area of interest
    through use of an adjustable bounding box.
    """

    print("\nPoint cloud max. xyz:", pcd.get_max_bound())
    print("Point cloud min. xyz:", pcd.get_min_bound())

    cbox = o3d.geometry.AxisAlignedBoundingBox(min_bound=pcd.get_max_bound(),
                                               max_bound=pcd.get_max_bound(),
                                               color=(1, 0, 0))

    # loop visualizations until user satisfied with cropping box
    while True:
        xmax = float(input("xmax = "))
        xmin = float(input("xmin = "))
        ymax = float(input("ymax = "))
        ymin = float(input("ymin = "))
        zmax = float(input("zmax = "))
        zmin = float(input("zmin = "))
        cmin = [xmin, ymin, zmin]
        cmax = [xmax, ymax, zmax]
        cbox = o3d.geometry.AxisAlignedBoundingBox(
            min_bound=cmin, max_bound=cmax)

        ax = o3d.geometry.TriangleMesh.create_coordinate_frame()
        ax.translate((xmin, ymin, zmin))
        o3d.visualization.draw_geometries(
            [pcd, cbox, ax], "Selected Area is within bounding box", vww, vwh)

        bound_q = input("Redefine bounding box? (y/n)"
                        "\n Enter (y) to redefine, (n) to continue or exit: ")
        if bound_q == "y":
            continue
        elif bound_q == "n":
            break
        else:
            print("Invalid user input.")

    cpcd = copy.deepcopy(pcd).crop(cbox)
    timestamp = time.strftime("%Y%m%d-%H%M%S")

    # crop-and-save bounded area, or exit
    while True:
        crop_q = input("Crop cloud to selected area and save it? (y/n)\n"
                       "Enter (y) to crop and save, (n) to exit: ")
        if crop_q == "y":
            pcd_name = f"CroppedPointCloud-{timestamp}.ply"
            o3d.io.write_point_cloud(pcd_name, cpcd)
        elif crop_q == "n":
            break
        else:
            print("Invalid user input.")
    return cpcd


# %% Execute

if __name__ == "__main__":

    pcd_file = "PointCloud-20210621-035537.ply"
    # pcd_file = input("Please select point cloud file to load.")

    print(f"\nLoading '{pcd_file}' point cloud...")
    tic = time.perf_counter()

    pcd = o3d.io.read_point_cloud(pcd_file)
    toc = time.perf_counter()
    print(f"Loaded in {toc - tic:0.4f} sec.\n")
    print(pcd_file, ":", pcd)


    box_or_lasso = input("\nTo select area by specific bounds, enter (1).\n"
                         "To select area manually, enter (2).\n")
    if box_or_lasso == "1":
        crop_pcd_area(pcd)
    elif box_or_lasso == "2":
        print("Press H to print help message.")
        print("Press [/] to increase/decrease field of view.")
        print("Press 'X'/'Y'/'Z' to enter orthogonal view along axis, "
              "   press again to flip.")
        print("Press 'K' to lock screen and to switch to selection mode:")
        print("   Drag for rectangle selection,")
        print("   or use ctrl + left click for polygon selection")
        print("Press 'C' to get a selected geometry and to save it.")
        print("Press 'F' to switch to freeview mode.")
        print("Press 'Q' to close the window.")
        o3d.visualization.draw_geometries_with_editing(
            [pcd], "Manual area selection", vww, vwh)
    else:
        print("Invalid user input.")

    input("Enter any key to exit.")

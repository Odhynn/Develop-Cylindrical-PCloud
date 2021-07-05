# -*- coding: utf-8 -*-

import copy

import open3d as o3d

def create_line(a, b, colour=[0, 0, 0]):
    """
    Creates a line segment.
    ---------------------
    a, b = points, 3-element array containing x,y,z
    colour = 3-element array containing r,g,b (normalized to [0,1] range )
    """

    # create cylinder axis at work area
    line_points = a, b
    line_lines = [[0, 1]]
    line_colour = [colour for i in range(len(line_lines))]

    line = o3d.geometry.LineSet()
    line.points = o3d.utility.Vector3dVector(line_points)
    line.lines = o3d.utility.Vector2iVector(line_lines)
    line.colors = o3d.utility.Vector3dVector(line_colour)
    return line


def rotate_line(xk, yk, zmin, zmax, rot_matrix, colour=[0, 0, 1]):
    """
    Rotates a vertical line (parallel to z-axis) around a point on the xy plane
    --------------------------------------
    xk, yk : coordinates of the line's projection on the xy plane
    zmin, zmax : lower, upper coordinates
    rot_matrix : euler rotation matrix (3x3 numpy array)
    """

    line = create_line([xk, yk, zmin], [xk, yk, zmax], colour)
    line.rotate(rot_matrix, center=[xk, yk, 0])
    return line


def cyl_mesh(pcd, cyl_p, extra_roof=0, thresh_mesh=False):
    """
    Visualizes a cylinder mesh around a corresponding cylindrical point cloud
    ---------------------------------
    pcd : point cloud (.ply)
    cyl_p : 6-element list, estimated cylinder parametres
        cyl_p[0] = xk, x coordinate of the cylinder centre (m)
        cyl_p[1] = xk, y coordinate of the cylinder centre (m)
        cyl_p[2] = omega, rotation angle about the x-axis (rad)
        cyl_p[3] = phi, rotation angle about the y-axis (rad)
        cyl_p[4] = r, radius of the cylinder (m)
        cyl_p[5] = RyRx, auxiliary rotation matrix (3x3 numpy array)

    extra_roof : by default the cylinder roof is set at the cloud's highest z
            so it may be visually checked that the independently shaped axis
            indeed passes through the roof center where the wires converge
        increase this to set the roof higher so that the cbox crops it off

    thresh_mesh : if <False>, the function returns a standard visualization
            if <True>, the function is used to convey the threshold cutoff,
                it takes an already modified cyl_p[4] = r +/- threshold
                and returns a red guide cylinder wireframe
                beyond which no points will survive the Thresholding
    """

    xyz_max = pcd.get_max_bound()
    xyz_min = pcd.get_min_bound()

    # create cylinder at (0, 0, 0); it extends to both z-directions from there
    mesh_cylinder = o3d.geometry.TriangleMesh.create_cylinder(
        radius=cyl_p[4], height=((2*xyz_max[2] + extra_roof)),
        resolution=32, split=round(4*xyz_max[2]))
    # ^ adjust resolution if the radial mesh lines are too thick or sparse
    mesh_cylinder.paint_uniform_color([0.1, 0.9, 0.1])

    # move cylinder to (xk, yk, 0)
    mesh_cylinder.translate((cyl_p[0], cyl_p[1], 0), relative=False)

    # rotate cylinder
    mesh_cylinder.rotate(
        mesh_cylinder.get_rotation_matrix_from_xyz((cyl_p[2], cyl_p[3], 0)))

    # a bounding box (slightly larger than the cloud) to crop the cylinder to
    cbox = o3d.geometry.AxisAlignedBoundingBox(
        min_bound=xyz_min-[1, 1, 1], max_bound=xyz_max+[1, 1, 1])

    # crop the cylinder so that visualization pans directly to work area
    mesh_cropped = copy.deepcopy(mesh_cylinder).crop(cbox)

    # create wireframe from mesh and visualize (or return thresh mesh)
    wire_cylinder = o3d.geometry.LineSet.create_from_triangle_mesh(
        mesh_cropped)

    # colour cylinder differently based on whether it will depict
    if thresh_mesh == False:  # green for fitted cylinder
        wire_cylinder.paint_uniform_color([0, 0.8, 0.2])
    else:  # red, the blood of angry thresholds
        wire_cylinder.paint_uniform_color([1, 0, 0])

    return wire_cylinder

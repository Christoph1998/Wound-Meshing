import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
from pyntcloud import PyntCloud
from scipy.spatial import ConvexHull, Delaunay
from functools import reduce
import math


def calc_max_distances(pcd):

    # read pcd as np array
    points = np.asarray(pcd.points)
    
    # define vars
    x_min, x_max, x_dist = 0, 0, 0
    y_min, y_max, y_dist = 0, 0, 0
    z_min, z_max, z_dist = 0, 0, 0

    # calc min
    x_min = np.min(points[:,0])
    y_min = np.min(points[:,1])
    z_min = np.min(points[:,2])

    # calc max
    x_max = np.max(points[:,0])
    y_max = np.max(points[:,1])
    z_max = np.max(points[:,2])

    # cals dist
    x_dist = x_max-x_min
    y_dist = y_max-y_min
    z_dist = z_max-z_min

    min_values = [x_min, y_min, z_min]
    max_values = [x_max, y_max, z_max]

    # return distances
    return x_dist, y_dist, z_dist


# @BRIEF: Function to crop a PointCloud to a specific area 

# @INPUT: PointCloud; COORDINATES for the cutting; filename for cropped PointCloud

# @RETURN: Cropped PointCloud
def crop_point_cloud(pcd, coordinates, plot=True, filename=''):

    # read coordinates min & max coordinates for cropping from coordinates array
   
    XMIN = coordinates[0]
    XMAX = coordinates[1]

    YMIN = coordinates[2]
    YMAX = coordinates[3]
    
    ZMIN = coordinates[4]
    ZMAX = coordinates[5]

    # write PointCloud as Numpy Array
    points = np.asarray(pcd.points)
    
    # crop z axis
    mask_z =np.where((points[:,2] > ZMIN) & (points[:,2] < ZMAX))[0] 
    points_z = points[mask_z]

    # crop y axis
    mask_y = np.where((points_z[:,1] > YMIN) & (points_z[:,1] < YMAX))[0]
    points_z_y = points_z[mask_y]

    # crop x axis
    mask_x = np.where((points_z_y[:,0] > XMIN) & (points_z_y[:,0] < XMAX))[0]
    points_z_y_x = points_z_y[mask_x]

    # create new Point Cloud from cropped coordinates
    cropped_pcd = o3d.geometry.PointCloud()
    cropped_pcd.points = o3d.utility.Vector3dVector(points_z_y_x)

    if plot:
        # draw the cropped PointCloud
        o3d.visualization.draw_geometries([cropped_pcd], window_name='cropped_pcd')

    if filename != '':
        # safe cropped PointCloud as file
        o3d.io.write_point_cloud(str(filename), cropped_pcd)

    return cropped_pcd

# @BRIEF: Reads a PointCloud from file, transforms it and plots
def read_point_cloud_from_file(path, transform='', plot=False):
    # read input point cloud
    pcd = o3d.io.read_point_cloud(path)

    # need inversion to get correct map. Depends on save() function in generation file
    if transform != '':
        pcd.transform(transform) 
    
    print("PointCloud successfully loaded and transformed")

    if plot:
        o3d.visualization.draw_geometries([pcd], window_name=path)

    return pcd
        

def create_mesh_bpa_algo(pcd, simplify_quadric_decimation, filename='', plot=True):
    # estimates normals
    pcd.estimate_normals()
    
    # calculate nearest neighbords
    distances = pcd.compute_nearest_neighbor_distance()
    
    # calc mean
    avg_dist = np.mean(distances)

    # calc radius
    radius = 3 * avg_dist

    # create the mesh using ball pivot algorithm
    bpa_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(pcd,o3d.utility.DoubleVector([radius, radius * 2]))

    # simplify the mesh
    simplified_mesh = bpa_mesh.simplify_quadric_decimation(simplify_quadric_decimation)

    # check consistency
    simplified_mesh.remove_degenerate_triangles()
    simplified_mesh.remove_duplicated_triangles()
    simplified_mesh.remove_duplicated_vertices()
    simplified_mesh.remove_non_manifold_edges()

    if filename != '':
        print("PCD wid gespeichert")
        o3d.io.write_triangle_mesh(filename+".ply", simplified_mesh)

    if plot:
        o3d.visualization.draw_geometries([simplified_mesh], window_name="mesh")

    return simplified_mesh

# calculate convex hull and get measurements
def compute_convex_hull(pcd, plot=True):
    # calculate convex_hull using o3d for graphic output
    hull, return_list = pcd.compute_convex_hull()
    hull_ls = o3d.geometry.LineSet.create_from_triangle_mesh(hull)
    hull_ls.paint_uniform_color((1, 0, 0))
    
    # create figure and create subplot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # create convex hull from pcd using scipy for measurement calculation
    points = np.asarray(pcd.points)
    fig_hull = ConvexHull(points)
    
    # get volume of defined object
    vol_mm = fig_hull.volume
    vol_m3 = vol_mm * 1e-9
    print("Volume = " + str(vol_mm) + " mm^3" + " = " + str(vol_m3) + " m^3")

    # get area from defined object
    area_mm = fig_hull.area
    area_m3 = area_mm * 1e-9
    print("Area = " + str(area_mm) + " mm^3" + " = " + str(area_m3) + " m^3")

    # get maximal distances in each coordinate
    x_dist_max, y_dist_max, z_dist_max = calc_max_distances(pcd)

    print("x_max_dist=", x_dist_max, "mm")
    print("y_max_dist=", y_dist_max, "mm") 
    print("z_max_dist=", z_dist_max, "mm")

    # get edges from pointcloud for plotting
    edges = list(zip(*points))

    for i in fig_hull.simplices:
        plt.plot(points[i,0], points[i,1], points[i,2], 'r-')

    ax.plot(edges[0],edges[1],edges[2],'bo')

    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')

    # plot
    if plot:
        plt.show()

    # return o3d convex hull ls for further graphic comparison
    return hull_ls

# function to plot pcd, mesh and convex hull in different combinations
def plot_wound(pcd=None, mesh=None, hull=None):
    
    # create list for elements which are going to be plot
    plot_list = []

    # check input parameters and add object to plot_list
    if pcd is not None:
        plot_list.append(pcd)
    if mesh is not None:
        plot_list.append(mesh)
    if hull is not None:
        plot_list.append(hull)

    # plot
    print("Plotting " + str(plot_list))
    o3d.visualization.draw_geometries(plot_list)



def plot_measurements(pcd):  
    # create figure and create subplot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # create convex hull from pcd using scipy for measurement calculation
    points = np.asarray(pcd.points)
    fig_hull = ConvexHull(points)
    
    # get edges from pointcloud for plotting
    edges = list(zip(*points))

    for i in fig_hull.simplices:
        plt.plot(points[i,0], points[i,1], points[i,2], 'r-')

    ax.plot(edges[0],edges[1],edges[2],'bo')

    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')

    # plot
    plt.show()


def create_mesh_poisson_algo(pcd, plot=False):
    pcd.compute_convex_hull()
    pcd.estimate_normals()
    pcd.orient_normals_consistent_tangent_plane(10)

    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=10, width=0, scale=20, linear_fit=True)[0]
    mesh.compute_vertex_normals()
    mesh.paint_uniform_color([0.5, 0.5, 0.5])
    mesh.remove_degenerate_triangles()
    if plot:
        o3d.visualization.draw_geometries([mesh], mesh_show_back_face=True)

    return mesh


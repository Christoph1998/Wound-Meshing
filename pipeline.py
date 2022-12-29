"""
@AUTHOR: TIMON SKUBAT & CHRISTOPH ZÖLL 

@PROJECT: WOUND MESHING PIPELINE 

@TIME: WISE 22/23
"""

import open3d as o3d
import numpy as np
from PyQt6 import uic
from PyQt6.QtWidgets import QApplication


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
def read_point_cloud_from_file(path, transform, plot):
    # read input point cloud
    pcd = o3d.io.read_point_cloud(r"SampleData\spatials_14-11-2022_14-42-13.ply")

    # need inversion to get correct map. Depends on save() function in generation file
    pcd.transform([[1,0,0,0],[0,-1,0,0],[0,0,1,0],[0,0,0,1]]) 
    
    print("PointCloud successfully loaded and transformed")

    if plot:
        o3d.visualization.draw_geometries([pcd], window_name=path)

    return pcd
        

def create_mesh_bpa_algo(pcd, simplify_quadric_decimation = 500000, filename='', plot=True):
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


if __name__ == "__main__":
    
    # read point cloud and prepare for further operations
    pcd = read_point_cloud_from_file(path="SampleData\spatials_14-11-2022_14-42-13.ply", 
                                transform=[[1,0,0,0],[0,-1,0,0],[0,0,1,0],[0,0,0,1]], plot=False)


    # define wound coordinates ->> (x_min, x_max, y_min, y_max, z_min, z_max)
    obj1 = (45, 70, 0, 25, 300, 355)
    obj2 = ()
    obj3 = ()

    # crop pcd
    pcd_obj_1 = crop_point_cloud(pcd=pcd, coordinates=obj1, plot=True, filename='cropped_point_cloud_onj1.ply')

    # create mesh using ball pivot algorithm
    # if filename != '' then pcd gets safed as file
    mesh = create_mesh_bpa_algo(pcd_obj_1, 1000000, filename='test', plot=True)
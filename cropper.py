# script to crop out unnassarry stuff and pick intresting objects

import cv2
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import open3d as o3d
import tkinter
import tkinter.filedialog as tkFileDialog
import math
from PIL import Image

# Helper Function to display two point clouds in one graph
def display_inlier_outlier(cloud, ind):
    inlier_cloud = cloud.select_by_index(ind)
    outlier_cloud = cloud.select_by_index(ind, invert=True)

    print("[INFO] Showing outliers (red) and inliers (gray): ")
    outlier_cloud.paint_uniform_color([1,0,0])
    inlier_cloud.paint_uniform_color([0.8,0.8,0.8])
    print("Plot inline and Outliner")
    o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud], zoom=0.3412, front=[0.426, -0.2125, -0.8795], lookat=[2.6172, 2.0475, 1.532],up=[-0.0694, -0.9768, 0.2024])



if __name__ == "__main__":

    # Reading point cloud from .ply file
    pcd_load = o3d.io.read_point_cloud(r"SampleData\spatials_14-11-2022_14-42-13.ply")
    pcd_load.transform([[1,0,0,0],[0,-1,0,0],[0,0,1,0],[0,0,0,1]]) # need inversion to get correct map. Depends on save() function in generation file
    print("Loaded point cloud: ")
    print(pcd_load)
    # pcd_load_cropped = o3d.geometry.crop_point_cloud(pcd_load,min_bound=np.arra([-math.inf, -math.inf, 100,]), max_bound=np.array([math.inf, math.inf, 350]))
    points = np.asarray(pcd_load.points)
    # pcd_load_cropped = pcd_load.select_by_index(np.where(points[:,2] < ZMAX)[0])
    # pcd_load_cropped = pcd_load.select_by_index(np.where(points[:,2] > ZMIN)[0])
    
    
    print(points)


    # Lötlitze Coordinates
    XMIN = 45
    XMAX = 70

    YMIN = 0
    YMAX = 25

    ZMIN = 332
    ZMAX = 355

    # crop z axis
    mask_z =np.where((points[:,2] > ZMIN) & (points[:,2] < ZMAX))[0] 
    points_z = points[mask_z]

    # crop y axis
    mask_y = np.where((points_z[:,1] > YMIN) & (points_z[:,1] < YMAX))[0]
    points_z_y = points_z[mask_y]

    mask_x = np.where((points_z_y[:,0] > XMIN) & (points_z_y[:,0] < XMAX))[0]
    points_z_y_x = points_z_y[mask_x]

    cropped_pcd = o3d.geometry.PointCloud()
    cropped_pcd.points = o3d.utility.Vector3dVector(points_z_y_x)

    o3d.visualization.draw_geometries([cropped_pcd], window_name='timon_and_christoph_croppen')

    o3d.io.write_point_cloud("cropped_pcd_loetlitze_only.ply", cropped_pcd)

    # display_inlier_outlier(pcd_load,mask)



  
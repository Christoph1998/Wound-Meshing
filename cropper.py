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
    
    # Parameter 
    CAMERA_INTRINSICS = np.array([[797.965087890625, 0.0, 630.5733642578125], [0.0, 797.965087890625, 389.7864685058594], [0.0, 0.0, 1.0]],dtype=np.float64)
    REMOVAL = "radius" # "statistical"
    ZMIN = 100 # mm
    ZMAX = 355 # mm 

    # load test arrays
    depth_map_in = np.load(r"SampleData\depth_14-11-2022_14-42-13.npy")
    disparity_map_in = np.load(r"SampleData\disparity_14-11-2022_14-42-13.npy")
    left_frame_in = np.load(r"SampleData\left_14-11-2022_14-42-13.npy")
    right_frame_in = np.load(r"SampleData\right_14-11-2022_14-42-13.npy")
    RGB_frame_in = np.load(r"SampleData\RGB_14-11-2022_14-42-13.npy")
    points_in = np.load(r"SampleData\spatials_14-11-2022_14-19-11.npy")

    # Analyse the given depth map
    print(f"Data-type of depth_map_in: {depth_map_in.dtype}")
    print(f"Shape of depth_map_in: {depth_map_in.shape}")
    print(f"Number of Dimensions of depth_map_in: {depth_map_in.ndim}")
    print(f"Unique values of depht_map_in: {pd.unique(np.concatenate(depth_map_in))}")
    print(f"Min value: {np.min(depth_map_in)}")
    print(f"Max value: {np.max(depth_map_in)}")

    # TODO: If general normalization is wanted, do it here!

    # Crop depth map to view only the single test objects
    depth_map_tesa = depth_map_in[300:480,700:870]
    depth_map_ell = depth_map_in[190:330,570:750]
    depth_map_iso = depth_map_in[150:430,300:600]

    depth_map_objects = [depth_map_tesa, depth_map_ell, depth_map_iso]
    objects = 3

    # Plot the normalized depth map, RGB frame and the test objects
    rows = 1
    cols = objects +2

    depth_map_vis = cv2.normalize(depth_map_in, None, 255, 0, cv2.NORM_INF, cv2.CV_8UC1)
    depth_map_vis = cv2.equalizeHist(depth_map_vis)
    depth_map_vis = cv2.applyColorMap(depth_map_vis,cv2.COLORMAP_HOT)

    fig, axs =plt.subplots(rows, cols)
    fig.suptitle('3 relevant objects in the depth_map')
    for i,img in enumerate(depth_map_objects):
        # Normalization of the depth_map for visualization purposes
        img = cv2.normalize(img, None, 255, 0, cv2.NORM_INF, cv2.CV_8UC1)
        img = cv2.equalizeHist(img)
        img = cv2.applyColorMap(img,cv2.COLORMAP_HOT) # Change the color maps (available maps see in docs of opencv); HOT or JET
        object = axs[i].imshow(img)
        #fig.colorbar(object,ax=axs[i], orientation='vertical', fraction=0.1)
    RGB_frame_in = cv2.cvtColor(RGB_frame_in,cv2.COLOR_BGR2RGB)
    axs[3].imshow(RGB_frame_in)
    axs[4].imshow(depth_map_vis)
    fig.subplots_adjust(wspace=0.5)

    plt.show()

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

    XMIN = 20
    XMAX = 300

    YMIN = -25
    YMAX = 40


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


    # display_inlier_outlier(pcd_load,mask)



  
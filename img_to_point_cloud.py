import open3d as o3d
import numpy as np

rgbd_img = o3d.data.BedroomRGBDImages()

pcd = o3d.geometry.create_point_cloud_from_rgbd_image(rgbd_img, )


pcd = o3d.geometry.create_point_cloud_from_rgbd_image(rgbd_img)
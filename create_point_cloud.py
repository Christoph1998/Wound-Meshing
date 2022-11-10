import open3d as o3d
import numpy as np

from matplotlib import pyplot as plt


'''
create RGBD image based on SUN dataset
http://www.open3d.org/docs/latest/tutorial/Basic/rgbd_image.html#SUN-dataset
https://rgbd.cs.princeton.edu/
'''
def create_rgbd_image_from_jpg_and_png(jpg_img, png_img):
    
    # read raw color data from JPG 
    color_raw = o3d.io.read_image(jpg_img)

    # read raw depth data from PNG
    depth_raw = o3d.io.read_image(png_img)    

    # create RGBD from SUN format
    rgbd_img = o3d.geometry.RGBDImage.create_from_sun_format(color_raw, depth_raw)

    return(rgbd_img)

'''
simple plot function for
'''
def plot_sun_grayscale_and_depth_image(rgbd_img):
    plt.subplot(1, 2, 1)
    plt.title('SUN grayscale image')
    plt.imshow(rgbd_img.color)
    plt.subplot(1, 2, 2)
    plt.title('SUN depth image')
    plt.imshow(rgbd_img.depth)
    plt.show()



if __name__ == "__main__":

    # read JPG and PNG from SUNRGBD.zip\SUNRGBD\kv2\kinect2data\000135_2014-05-20_17-03-38_260595134347_rgbf000100-resize\
    jpg_img = "ImageData/0000100.jpg"
    png_img = "ImageData/0000100.png"

    # create RGBD
    rgbd_img = create_rgbd_image_from_jpg_and_png(jpg_img, png_img)

    # print information
    print(rgbd_img)

    # plot RGBD image
    plot_sun_grayscale_and_depth_image(rgbd_img)

    # create PCD from RGBD    
    intrinsics = o3d.camera.PinholeCameraIntrinsic(o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault)
    
    # load extrincis from SUN-Dataset (optional parameter)
    # extrinsics = np.loadtxt("ImageData\extrinsics.txt")

    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_img, intrinsics)
    # Flip it, otherwise the pointcloud will be upside down
    pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
    o3d.visualization.draw_geometries([pcd])


    o3d.io.write_point_cloud("pcd_output.pcd", pcd)


    print("Ende")
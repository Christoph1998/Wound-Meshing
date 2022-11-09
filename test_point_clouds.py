import open3d as o3d
import numpy as np

if __name__ == "__main__":

    pcd = o3d.io.read_point_cloud("pcd_output.pcd")

    print("coord")
    print(np.asarray(pcd.points))


    print("Colors")
    print(np.asarray(pcd.colors))

    pcd.estimate_normals()

    print("normals")
    print(np.asarray(pcd.normals))



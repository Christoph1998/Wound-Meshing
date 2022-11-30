#5steps to create mesh
import numpy as np
import open3d as o3d


if __name__ == "__main__":

    pcd = o3d.io.read_point_cloud("cropped_pcd_loetlitze.ply")
    pcd.estimate_normals()


    print("compute mesh")
    poisson_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=8, width=0, scale=1.1, linear_fit=False)[0]


    with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
        mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=9)

    print("Draw")
    o3d.visualization.draw_geometries([pcd, mesh])

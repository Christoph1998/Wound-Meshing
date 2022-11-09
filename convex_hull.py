#5steps to create mesh
import numpy as np
import open3d as o3d

def write_pcd_to_txt(pcd):
    temp = np.column_stack([pcd.points, pcd.colors, pcd.normals])
    np.savetxt("pcd_as_txt.txt", temp)


if __name__ == "__main__":

    print("Read pointcloud and draw")
    pcd = o3d.io.read_point_cloud("pcd_output.pcd")
    # o3d.visualization.draw_geometries([pcd])

    # print("Estimate normals - this is optiobnal - test it")
    # pcd.estimate_normals()


    print("Start strategy")

    hull, return_list = pcd.compute_convex_hull()
    print(return_list)
    hull_ls = o3d.geometry.LineSet.create_from_triangle_mesh(hull)
    hull_ls.paint_uniform_color((1, 0, 0))
    o3d.visualization.draw_geometries([pcd, hull_ls])

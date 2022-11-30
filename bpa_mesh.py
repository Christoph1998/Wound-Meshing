#5steps to create mesh
import numpy as np
import open3d as o3d
from open3d import geometry as o3dgeo

def write_pcd_to_txt(pcd):
    temp = np.column_stack([pcd.points, pcd.colors, pcd.normals])
    np.savetxt("pcd_as_txt.txt", temp)


if __name__ == "__main__":

    print("Read pointcloud and draw")
    # pcd = o3d.io.read_point_cloud("pcd_output.pcd")
    pcd = o3d.io.read_point_cloud("cropped_pcd_loetlitze.ply")
   # o3d.visualization.draw_geometries([pcd])

    print("Estimate normals - this is optiobnal - test it")
    pcd.estimate_normals()

   # write_pcd_to_txt(pcd)

    # o3d.visualization.draw_geometries([pcd])

    print("Start strategy")

    # strategie 1 - ball pivot algorithm
    distances = pcd.compute_nearest_neighbor_distance()
    avg_dist = np.mean(distances)
    radius = 3 * avg_dist

    print("Create bpa mesh")
    bpa_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(pcd,o3d.utility.DoubleVector([radius, radius * 2]))

    print("create dec_mesh and remove shit")
    # anzahl an dreiecken festlegen mit denen das mesh gebildet wird
    dec_mesh = bpa_mesh.simplify_quadric_decimation(1000000)

    # Konsistenz des meshes überprüfen
    dec_mesh.remove_degenerate_triangles()
    dec_mesh.remove_duplicated_triangles()
    dec_mesh.remove_duplicated_vertices()
    dec_mesh.remove_non_manifold_edges()


    #Exportieren
    print("export mesh")
    o3d.io.write_triangle_mesh("dec_mesh.ply", dec_mesh)
    o3d.io.write_triangle_mesh("dec_mesh.obj", dec_mesh)

    # calculate convex_hull
    hull, return_list = pcd.compute_convex_hull()
    hull_ls = o3d.geometry.LineSet.create_from_triangle_mesh(hull)
    hull_ls.paint_uniform_color((1, 0, 0))


    print("Draw_bpa")
    # o3d.visualization.draw_geometries([bpa_mesh])
    print("Draw dec")
    o3d.visualization.draw_geometries([pcd, dec_mesh, hull_ls])





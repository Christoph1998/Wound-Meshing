#5steps to create mesh
import numpy as np
import open3d as o3d
from pyntcloud import PyntCloud
import matplotlib.pyplot as plt
from scipy.spatial import ConvexHull, convex_hull_plot_2d
from mpl_toolkits.mplot3d import Axes3D

def write_pcd_to_txt(pcd):
    temp = np.column_stack([pcd.points, pcd.colors, pcd.normals])
    np.savetxt("pcd_as_txt.txt", temp)


if __name__ == "__main__":

    print("Read pointcloud and draw")
    # pcd = o3d.io.read_point_cloud("pcd_output.pcd")
    pcd = o3d.io.read_point_cloud("cropped_pcd_loetlitze_only.ply")
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
    
    # Calculate vol via https://github.com/isl-org/Open3D/issues/2439
    cloud = PyntCloud.from_instance("open3d", dec_mesh)
    convex_hull_id = cloud.add_structure("convex_hull")
    convex_hull = cloud.structures[convex_hull_id]


    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    points = np.asarray(pcd.points)

    fig_hull = ConvexHull(points)
    
    vol_mm = fig_hull.volume
    vol_m3 = vol_mm * 1e-9
    print("Volume = " + str(vol_mm) + " mm^3" + " = " + str(vol_m3) + " m^3")

    edges = list(zip(*points))

    for i in fig_hull.simplices:
        plt.plot(points[i,0], points[i,1], points[i,2], 'r-')

    ax.plot(edges[0],edges[1],edges[2],'bo')

    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')

    plt.show()

    print("Draw_bpa")
    # o3d.visualization.draw_geometries([bpa_mesh])
    print("Draw dec")
    o3d.visualization.draw_geometries([pcd, dec_mesh, hull_ls])
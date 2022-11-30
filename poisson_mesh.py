#5steps to create mesh
import numpy as np
import open3d as o3d

def lod_mesh_export(mesh, lods, extension, path):
    mesh_lods={}
    for i in lods:
        mesh_lod = mesh.simplify_quadric_decimation(i)
        o3d.io.write_triangle_mesh(path+"lod_"+str(i)+extension, mesh_lod)
        mesh_lods[i]=mesh_lod
    print("generation of "+str(i)+" LoD successful")
    return mesh_lods


if __name__ == "__main__":

    # pcd = o3d.io.read_point_cloud("pcd_output.pcd")

    #pointcloud transformation (nur für numpy klassen notwendig)(schauen, was parameter anordnung verändert)
    point_cloud= np.loadtxt("sample_w_normals.txt",skiprows=1)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(point_cloud[:,:3])
    pcd.colors = o3d.utility.Vector3dVector(point_cloud[:,3:6]/255)
    pcd.normals = o3d.utility.Vector3dVector(point_cloud[:,6:9])

    o3d.visualization.draw_geometries([pcd])

    print("compute mesh")
    poisson_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=8, width=0, scale=1.1, linear_fit=False)[0]

    print("Cropping")
    bbox = pcd.get_axis_aligned_bounding_box()
    p_mesh_crop = poisson_mesh.crop(bbox)

    #Exportieren
    print("export mesh")
    o3d.io.write_triangle_mesh("p_mesh_c.ply", p_mesh_crop)
    o3d.io.write_triangle_mesh("p_mesh_c.obj", p_mesh_crop)

    # my_lods = lod_mesh_export(bpa_mesh, [100000,50000,10000,1000,100], ".ply", "./Wound-Meshing/")

    # o3d.visualization.draw_geometries([my_lods[100]])

    print("Draw")
    o3d.visualization.draw_geometries([p_mesh_crop])

import open3d as o3d
import numpy as np

# image processing
print("Testing IO for images ...")
# image_data = o3d.data.JuneauImage()
# img = o3d.io.read_image(image_data.path)
img = o3d.io.read_image("timon_chris.jpg")
print(img)
o3d.io.write_image("cooles Bild", img)

# point cloud processing
print("Testing IO for point cloud ...")
sample_pcd_data = o3d.data.PCDPointCloud()
pcd = o3d.io.read_point_cloud(sample_pcd_data.path)
print(pcd)
o3d.io.write_point_cloud("coole point cloud.pcd", pcd)
print(np.asarray(pcd.points))
o3d.visualization.draw_geometries([pcd],
                                  zoom=0.3412,
                                  front=[0.4257, -0.2125, -0.8795],
                                  lookat=[2.6172, 2.0475, 1.532],
                                 up=[-0.0694, -0.9768, 0.2024])

# point mesh cloud
print("Testing IO for meshes ...")
knot_data = o3d.data.KnotMesh()
mesh = o3d.io.read_triangle_mesh(knot_data.path)
print(mesh)
o3d.io.write_triangle_mesh("copy_of_knot.ply", mesh)

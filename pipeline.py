"""
@AUTHOR: TIMON SKUBAT & CHRISTOPH ZÖLL 

@PROJECT: WOUND MESHING PIPELINE 

@TIME: WISE 22/23
"""

from wound_meshing import *

def example_for_object_1():
    # read point cloud and prepare for further operations
    pcd = read_point_cloud_from_file(path="SampleData\spatials_14-11-2022_14-42-13.ply", 
                                transform=[[1,0,0,0],[0,-1,0,0],[0,0,1,0],[0,0,0,1]], plot=False)


    # define wound coordinates ->> (x_min, x_max, y_min, y_max, z_min, z_max)
    obj = (45, 70, 0, 25, 300, 355)

    # crop pcd
    pcd = crop_point_cloud(pcd=pcd, coordinates=obj, plot=False, filename='cropped_point_cloud_onj1.ply')

    # create mesh using ball pivot algorithm
    # if filename != '' then pcd gets safed as file
    mesh = create_mesh_bpa_algo(pcd, 1000000, filename='test', plot=False)

    # compute convex hull and calculate object measurements
    hull = compute_convex_hull(pcd = pcd, plot=False)

    # plot the calculated forms in diffrent combinations
    plot_wound(pcd=pcd, mesh=mesh, hull=hull)

def example_for_object_2():
    # read point cloud and prepare for further operations
    pcd = read_point_cloud_from_file(path="SampleData\spatials_14-11-2022_14-42-13.ply", 
                                transform=[[1,0,0,0],[0,-1,0,0],[0,0,1,0],[0,0,0,1]], plot=False)

    plot_wound(pcd)

    # define wound coordinates ->> (x_min, x_max, y_min, y_max, z_min, z_max)
    #     obj = (45, 70, 0, 25, 300, 355)
    obj = (0, 150, -150, -50, 300, 355)

    # crop pcd
    pcd = crop_point_cloud(pcd=pcd, coordinates=obj, plot=True, filename='cropped_point_cloud_onj1.ply')

    # create mesh using ball pivot algorithm
    # if filename != '' then pcd gets safed as file
    # mesh = create_mesh_bpa_algo(pcd, 1000, filename='test', plot=False)

    # compute convex hull and calculate object measurements
    hull = compute_convex_hull(pcd = pcd, plot=False)

    # plot the calculated forms in diffrent combinations
    #plot_wound()


if __name__ == "__main__":
    example_for_object_1()

# - Entlötlitze: Radius=7.5mm, Tiefe=9mm, zylinderförmig
# - Tesa-Rolle: Radius_inner=13mm, Tiefe=20mm, zylinderförmig
# - Isolierband: Radius=24,5mm, Tiefe=22mm, zylinderförmig


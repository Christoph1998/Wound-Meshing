"""
@AUTHOR: TIMON SKUBAT & CHRISTOPH ZÖLL 

@PROJECT: WOUND MESHING PIPELINE 

@TIME: WISE 22/23
"""

from wound_meshing import *
import random

def example_for_object_1():
    # read point cloud and prepare for further operations
    pcd = read_point_cloud_from_file(path=r"SampleData\spatials_14-11-2022_14-42-13.ply", 
                               transform=[[1,0,0,0],[0,-1,0,0],[0,0,1,0],[0,0,0,1]], plot=False)

    # pcd = read_point_cloud_from_file(path=r"SampleData\spatials_14-11-2022_14-42-13.ply", plot=False)


    # define wound coordinates ->> (x_min, x_max, y_min, y_max, z_min, z_max)
    # with border
    # obj = (45, 70, 0, 25, 300, 355)

    # without border
    #obj = (40, 70, 0, 25, 332, 355)

    obj = (40, 70, 0, 25, 332, 400)

    # crop pcd
    pcd = crop_point_cloud(pcd=pcd, coordinates=obj, plot=False, filename='cropped_point_cloud_onj1.ply')

    plot_measurements(pcd)

    # create mesh using ball pivot algorithm
    # if filename != '' then pcd gets safed as file
    mesh = create_mesh_bpa_algo(pcd, 1000000, filename='test', plot=False)


    # mesh = create_mesh_poisson_algo2(pcd)
    

    volume_calculation(pcd)


    # compute convex hull and calculate object measurements
    hull = compute_convex_hull(pcd = pcd, plot=False)

    # plot the calculated forms in diffrent combinations
    plot_wound(pcd=pcd, mesh=mesh, hull=hull)

def testboard_v1_1200dots_35cm_flat_large():
    # read point cloud and prepare for further operations
    pcd = read_point_cloud_from_file(path=r"SampleData\spatials_12-01-2023_15-29-46.ply", 
            transform=[[1,0,0,0],[0,-1,0,0],[0,0,1,0],[0,0,0,1]], plot=False)


    # define wound coordinates ->> (x_min, x_max, y_min, y_max, z_min, z_max)

    # full wound plate
    # obj = (-100, 50, -20, 40, 390, 420)

    # object top row third
    # make use of plot_measurements to define coordinates easier
    obj=(-49,-27,-6.5,10,390,420)

    # crop pcd
    pcd = crop_point_cloud(pcd=pcd, coordinates=obj, plot=False, filename='cropped_point_cloud_onj1.ply')

    # plot_measurements(pcd)

    
    # create mesh using ball pivot algorithm
    # if filename != '' then pcd gets safed as file
    mesh = create_mesh_bpa_algo(pcd, 1000000, filename='test', plot=False)

    volume_calculation(pcd)

    # compute convex hull and calculate object measurements
    hull = compute_convex_hull(pcd = pcd, plot=False)

    # plot the calculated forms in diffrent combinations
    plot_wound(pcd=pcd, mesh=mesh, hull=hull)



def testboard_v1_1200dots_35cm_flat_large2():
    # read point cloud and prepare for further operations
    pcd = read_point_cloud_from_file(path=r"SampleData\spatials_12-01-2023_15-29-46.ply", plot=False)


    # define wound coordinates ->> (x_min, x_max, y_min, y_max, z_min, z_max)

    # full wound plate
    obj = (-100, 50, -20, 40, 390, 420)

    # object top row third
    # make use of plot_measurements to define coordinates easier
    #obj=(-49,-27,-6.5,10,390,420)
    # crop pcd
    pcd = crop_point_cloud(pcd=pcd, coordinates=obj, plot=False, filename='cropped_point_cloud_onj1.ply')

    # plot_measurements(pcd)

    plot_wound(pcd)
    # create mesh using ball pivot algorithm
    # if filename != '' then pcd gets safed as file
    mesh = create_mesh_bpa_algo(pcd, 1000000, filename='test', plot=False)

    volume_calculation(pcd)

    # compute convex hull and calculate object measurements
    hull = compute_convex_hull(pcd = pcd, plot=False)

    # plot the calculated forms in diffrent combinations
    plot_wound(pcd=pcd, mesh=mesh, hull=hull)


def presentation_examples():
    # data from Testboard_v1_1200dots_40cm
   # read point cloud and prepare for further operations
    pcd = read_point_cloud_from_file(path=r"SampleData\spatials_12-01-2023_15-29-46.ply", plot=False)
    

    # define wound coordinates ->> (x_min, x_max, y_min, y_max, z_min, z_max)

    # full wound plate
    #obj = (-100, 50, -20, 40, 390, 420)

    # object top row third
    # make use of plot_measurements to define coordinates easier
    obj=(-49,-27,-8,6,400,411)

    # crop pcd
    pcd = crop_point_cloud(pcd=pcd, coordinates=obj, plot=True, filename='cropped_point_cloud_onj1.ply')

    plot_measurements(pcd)

    #mesh = create_mesh_bpa_algo(pcd, 10000000, filename='test', plot=True)
    mesh=create_mesh_poisson_algo(pcd)
    plot_wound(mesh = mesh)

    #volume_calculation(pcd)

    # mesh = create_mesh_poisson_algo(pcd)

    # compute convex hull and calculate object measurements
    hull = compute_convex_hull(pcd = pcd, plot=False)

    # plot the calculated forms in diffrent combinations
    plot_wound(pcd=pcd, mesh=mesh, hull=hull)

if __name__ == "__main__":

    #testboard_v1_1200dots_35cm_flat_large()
    #example_for_object_1()
    presentation_examples()


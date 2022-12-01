# Wound-Meshing
Pipeline for point cloud and mesh creation

The goal of the project is to create and evaluate a mesh of a wound. The input consists of color and depth images and is described by a point cloud.
In the current phase, the pipeline consists of individual modules to analyze the processing of input images and to try out different methods.


Passing through the pipeline:

1. Load a Point Cloud
    For all other methods a pointcloud is used as input parameter. This can either be loaded directly or generated with the help of the script "create_point_cloud.py".

    - run "create_point_cloud.py"
    - INPUT: JPG (color), PNG (depth)
        - Input is processed to RGBD
        - RGBD is processed to PointCloud
    - RETURN: Point Cloud


2.  Cropping the point cloud to focus the wound. The cropped pointcloud must consist exclusively of the wound so as not to distort further body calculations. 
    Cropping of the point cloud is performed by the "cropper.py" script. 


3. Create mesh & calculate volume
    The script "bpa_mesh.py" is used to create the mesh. This uses the "Ball Pivot Algorithm" for mesh creation. Furthermore, the volume of the wound is calculated with the help of a convex envelope.


# Wound-Meshing
Pipeline for point cloud and mesh creation

The current version contains different part modules of the Wound Meshing Pipeline. We use different methods to analyze the different processing steps of the pipeline.


Please execute the code in the following order:

1. Create a Point Cloud
    - run "create_point_cloud.py"
    - INPUT: JPG (color), PNG (depth)
        - Input is processed to RGBD
        - RGBD is processed to PointCloud
    - RETURN: Point Cloud

2. Create Mesh
    For mesh creation we have to scripts:
    a) "bpa_mesh.py" using ball pivot algorithm
    b) "poisson_mesh.py" using poisson construction

    INPUT: Point Cloud
    RETURN: Mesh

3. Draw convex hull
    "convex_hull.py" draws a hull over point cloud surface    

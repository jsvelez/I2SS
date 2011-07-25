Compilation instructions (assuming you already have PCL 1.0.1 installed and a Unix machine):
1) Create a subdirectory named "build" wherever you have the source and CMakeLists.txt
2) Open a terminal, cd to the newly-created subdirectory and do "cmake .."
3) After that, just "make" should do the trick.

Usage: ./cluster_subtraction input_cloud.pcd distance_threshold
       Options: --sor mean_k stdev_threshold                         (default: 50 1)            [Parameters for statistic outlier removal*]
                --cluster tolerance min_points max_points            (default: 0.02 250 2500)   [Parameters for Euclidean cluster extraction]
                --method [ransac | mlesac | rmsac | rransac]         (default: ransac)          [Estimation method to use for segmentation]
                --model [plane | line | circle | sphere | cylinder   (default: plane)           [Model to segment out (default: plane)]
                   | normal_plane | parallel_plane | registration]
                --normal_weight nwt                                  (default: 0.1)             [Normal distance weight (for normal plane segmentation)]

       *Not needed in normal-based segmentation

       ./scale inputFile outputFile
       Options: none

       *Note: make sure the input file is completely clean, as it does not generate or ignore PCL file headers!

The parameters used to generate the resulting PCDs are in file params.doc.

To view all the generated pcd files together for comparison, just do: pcd_viewer <method_name>*.pcd

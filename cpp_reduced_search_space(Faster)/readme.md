Required packages :

1) OpenCV Latest (4.1.1)
2) Latest VTK libraries (8.2.0) for visualization  (OpenCV should be built with VTK option ON)

For Velodyne_capture :
1) Boost library (if using live sensor capture)

More info : https://github.com/UnaNancyOwen/VelodyneCapture

Usage :

./Cyl_filter ../../Dataset/points_68.bin 0.03 1 3

Velodyne capture:

1) cd /Sample/viewer/
2) make
3) ./viewer

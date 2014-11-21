##What is lsd-slam?##
LSD-SLAM is a direct, monocular SLAM package for ROS, released in 2014.
Go to their github page to get the code, details about how it works including publications, and a lot more help on how to use it.

https://github.com/tum-vision/lsd_slam

##Compilation hints##
- Although their readme says that they use rosmake, and not catkin, there is a working catkin branch. This is the one that I've been using.
- If you have a corei7 CPU, and Ubuntu 12.04, you will get "assembler instruction missing" errors when compiling lsd-slam.
You can fix these by replacing the compiler flag '-march=native' with `-march=corei7` in the CMakeLists.txt files for lsd_slam_core and
lsd_slam_viewer. The problem is due to an out of date gcc version in the Ubuntu 12.04 package repositories.

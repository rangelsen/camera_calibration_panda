# Volumetric reconstruction from depth image

This program is created to fuse depth images from an Intel RealSense SR300
camera mounted in the eye-in-hand configuration of a robotic manipulator.

The manipulator's workspace volume is represented as an octree where the
resolution of the octree is the length of the sides of the voxels at the
highest level. Both the number of levels and the resolution is calculated
based on the user-specified dimensions of the entire workspace volume and
a user-specified desired upper limit for the resolution.

![](https://github.com/rangelsen/VoxelSpaceReconstruction/blob/master/Screenshots/human1.png)
![](https://github.com/rangelsen/VoxelSpaceReconstruction/blob/master/Screenshots/human2.png)
![](https://github.com/rangelsen/VoxelSpaceReconstruction/blob/master/Screenshots/human3.png)

## Dependencies

This collection of software is built with the following dependencies, all of
which is open source and easily obtainable with an internet connection.

### Open Computer Vision 2
    
OpenCV is used to store and display images obtained with the RealSense SR300
camera in a reasonable way.

### Open Graphics Library
    
OpenGL is used to visualize the octree representation of the voxel grid.

### Simple Direct Media Layer 2

SDL is used for GPU context handling, window management, and as an input
hardware layer.

### OpenGL Mathematics

GLM is used as a lightweight, header-only math library that goes great with
OpenGL.

### Eigen

Eigen is the primary library used for mathematical computations

### librealsense

librealsense is used as the interface to the SR300 camera hardware


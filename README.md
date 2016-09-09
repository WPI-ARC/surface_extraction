# Surface Extraction

This project is an on-demand surface extraction framework designed for humanoid locomotion planning. Its key feature is the ability to incrementally process the scene in order to waste unnecessary effort processing areas of the environment for which surface information is not required.

## Setup

Surface Extraction runs on Ubuntu 16.04 and ROS Kinetic (http://wiki.ros.org/kinetic/Installation/Ubuntu)

1. Install PCL version 1.8. As of this writing PCL 1.8 is not available by PPA, so it must be installed from source as described here: http://pointclouds.org/documentation/tutorials/compiling_pcl_posix.php
2. Install CGAL with `apt-get install libcgal-dev`
3. Download the `arc_utilities` package and put it in your Catkin workspace
4. Download `surface_extraction` and put it in your Catkin workspace
5. Compile with `catkin_make`

## Usage


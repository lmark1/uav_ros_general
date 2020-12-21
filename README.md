# UAV ROS General

| Ubuntu 18.04  | Ubuntu 20.04|
|---------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------|
 [![Melodic](https://github.com/lmark1/uav_ros_general/workflows/Melodic/badge.svg)](https://github.com/lmark1/uav_ros_general/actions) | [![Noetic](https://github.com/lmark1/uav_ros_general/workflows/Noetic/badge.svg)](https://github.com/lmark1/uav_ros_general/actions) |

## Summary

This package contains various sensor configuration for the [uav_ros_stack](https://github.com/lmark1/uav_ros_stack).  

It is intended to be used on UAV platforms with an onboard computer running Ubuntu Linux.

## Optional Dependancies

Depending on the UAV sensor suite additional ROS driver packages may need to be installed:  

* [velodyne](https://github.com/ros-drivers/velodyne) - Collection of ROS packages supporting Velodyne high definition 3D LiDARs
* [ouster_os0_driver](https://github.com/larics/ouster_os0_driver/) - Collection of ROS packages supporting Ouster LiDARs
* [openzen_sensor](https://bitbucket.org/lpresearch/openzenros/src/master/) - This software allows to forward sensor data from sensor connected via OpenZen to ROS. In the context of this package it is used for the LPMS IMU sensor.
* [realsense2_camera](https://github.com/IntelRealSense/realsense-ros) - These are packages for using Intel RealSense cameras (D400 series SR300 camera and T265 Tracking Module) with ROS.
* [zed_cpu_ros](https://github.com/willdzeng/zed_cpu_ros) - A simple zed camera driver which only use CPU and only publish left and right raw images and its camera info.

The following SLAM software may need to be installed when using this package:

* [cartographer](https://github.com/larics/cartographer) & [cartographer_ros](https://github.com/larics/cartographer_ros) - Cartographer is a system that provides real-time simultaneous localization and mapping (SLAM) in 2D and 3D across multiple platforms and sensor configurations.
* [A-LOAM](https://github.com/larics/A-LOAM) - Advanced LiDAR Odometry And Mapping
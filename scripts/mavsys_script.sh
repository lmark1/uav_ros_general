#!/bin/bash
rosrun mavros mavsys --mavros-ns $UAV_NAMESPACE/mavros rate --all 10
sleep 1
roslaunch uav_ros_general mavsys.launch

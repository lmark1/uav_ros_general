cmake_minimum_required(VERSION 2.8.3)

project(uav_ros_general)

set(CMAKE_CXX_STANDARD 17)

find_package(
  catkin
  REQUIRED
  tf
  roscpp
  roslib
  rospack
  geometry_msgs
  uav_ros_lib)

catkin_package(INCLUDE_DIRS src CATKIN_DEPENDS roscpp roslib rospack)

include_directories(${EIGEN3_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} include
                    ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)

add_executable(map2transform nodes/tf2TransformStamped.cpp)
target_link_libraries(map2transform ${catkin_LIBRARIES})

add_executable(velocity_estimation_node nodes/VelocityEstimationNode.cpp)
target_link_libraries(velocity_estimation_node ${catkin_LIBRARIES})

add_executable(kalman_const_acc_node nodes/VelocityEstimationNode_ConstAcc.cpp)
target_link_libraries(kalman_const_acc_node ${catkin_LIBRARIES})

add_executable(kalman_const_acc_ukf_node nodes/VelocityEstimationNode_ConstAccUKF.cpp)
target_link_libraries(kalman_const_acc_ukf_node ${catkin_LIBRARIES})

add_executable(rc_to_joy_node nodes/rc_to_joy_node.cpp src/RcToJoy.cpp)
target_link_libraries(rc_to_joy_node ${catkin_LIBRARIES})

add_executable(rc_override_node nodes/rc_override_node.cpp src/RcOverride.cpp)
target_link_libraries(rc_override_node ${catkin_LIBRARIES})

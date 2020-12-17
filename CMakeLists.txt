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
  cmake_modules
  dynamic_reconfigure)
find_package(Eigen3 REQUIRED)

generate_dynamic_reconfigure_options(config/VelocityEstimationParameters.cfg
                                     config/KalmanConstAccParameters.cfg)

catkin_package(INCLUDE_DIRS src CATKIN_DEPENDS roscpp roslib rospack)

include_directories(${EIGEN3_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} include
                    ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)

add_executable(map2transform src/tf2TransformStamped.cpp)
target_link_libraries(map2transform ${catkin_LIBRARIES})

add_executable(velocity_estimation_node src/VelocityEstimationNode.cpp)
target_link_libraries(velocity_estimation_node ${catkin_LIBRARIES})
add_dependencies(velocity_estimation_node ${catkin_EXPORTED_TARGETS}
                 ${PROJECT_NAME}_gencfg)

add_executable(kalman_const_acc_node src/VelocityEstimationNode_ConstAcc.cpp)
target_link_libraries(kalman_const_acc_node ${catkin_LIBRARIES})
add_dependencies(kalman_const_acc_node ${catkin_EXPORTED_TARGETS}
                 ${PROJECT_NAME}_gencfg)

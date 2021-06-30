# ERL drone

## Map Post-processing

These instructions will help you with the post-processing of the map created by Google Cartographer.

### Post-processing of the online created map

After the mission call the service for finishing trajectory and the service which creates .pbstream file:

```
rosservice call /finish_trajectory 0

rosservice call /write_state "filename: '${HOME}/example.bag.pbstream' include_unfinished_submaps: true"
```

If you want to visualise a trajectory and a map without using .bag file:

```
rosrun cartographer_ros cartographer_offline_node -configuration_directory ~/catkin_ws/src/uav_ros_general/cartographer_config -configuration_basenames second_tuning.lua points2:=/velodyne_points imu:=/mavros/imu/data_raw fix:=/mavros/global_position/raw/fix -urdf_filenames= ~/catkin_ws/src/uav_ros_general/urdf/vlp16-imu.urdf -keep_running=true -load_frozen_state=false -load_state_filename=${HOME}/example.bag.pbstream
```

Then start assets_writer node with created .pbstream file:

```
rosrun cartographer_ros cartographer_assets_writer -configuration_directory ~/catkin_ws/src/uav_ros_general/cartographer_config -configuration_basename assets_writer.lua -urdf_filename= ~/catkin_ws/src/uav_ros_general/urdf/vlp16-imu.urdf -bag_filenames=${HOME}/example.bag -pose_graph_filename=${HOME}/example.bag.pbstream
```

Wait until the processing is over.
Now, .ply and .png files are generated.

* [MeshLab](http://www.meshlab.net/) - The system for .ply file visualisation
* [GoogleEarth](https://www.google.hr/intl/hr/earth/) - The program in which you can import .png file and set it in the right place on the Earth using LLA (Latitude, Longitude, Altitude) coordinates.

### Post-processing using .bag file

Start cartographer_offline node:

```
rosrun cartographer_ros cartographer_offline_node -configuration_directory ~/catkin_ws/src/uav_ros_general/cartographer_config -configuration_basenames second_tuning.lua points2:=/velodyne_points imu:=/mavros/imu/data fix:=/mavros/global_position/global -urdf_filenames=~/catkin_ws/src/uav_ros_general/urdf/vlp16-imu.urdf -keep_running=true -bag_filenames=${HOME}/example.bag

```
Cartographer_offline node creates .pbstream file in the folder where .bag file is.

Then start assets_writer node with created .pbstream file:

```
rosrun cartographer_ros cartographer_assets_writer -configuration_directory ~/catkin_ws/src/uav_ros_general/cartographer_config -configuration_basename assets_writer.lua -urdf_filename= ~/catkin_ws/src/uav_ros_general/urdf/vlp16-imu.urdf -bag_filenames=${HOME}/example.bag -pose_graph_filename=${HOME}/example.bag.pbstream
```
Wait until the processing is over.
Now, .ply and .png files are generated.
Use [MeshLab](http://www.meshlab.net/) and [GoogleEarth](https://www.google.hr/intl/hr/earth/) for visualisation.

### Creating and importing trajectory in Google Earth

Visualise trajectoy in RViz:

```
rosrun cartographer_ros cartographer_offline_node -configuration_directory ~/catkin_ws/src/uav_ros_general/cartographer_config -configuration_basenames second_tuning.lua points2:=/velodyne_points imu:=/mavros/imu/data_raw fix:=/mavros/global_position/raw/fix -urdf_filenames= ~/catkin_ws/src/uav_ros_general/urdf/vlp16-imu.urdf -keep_running=true -load_frozen_state=false -load_state_filename=${HOME}/example.bag.pbstream
```

Call /write_ecef_trajectory service and add output filename where x,y,z coordinates will be saved:

```
rosservice call /write_ecef_trajectory "filename: '${HOME}/first_try.txt'
```

Change the path in uav_ros_general/scripts/ecef_to_lla.py.

```
./ecef_to_llh.py > ${HOME}/first_try.kml
```

The .kml file is ready and now it can be opened in Google Earth.
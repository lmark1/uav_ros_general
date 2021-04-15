# OPTITRACK 

This is a guide for setting up Optitrack to play nicely with PX4 1.11.3.  
**NOTE** - Other Firmware versions may behave differently.  
A more in-depth guide can be found here: [PX4 - External Position Estimataion](https://docs.px4.io/master/en/ros/external_position_estimation.html)

## Why is this setup needed?

*Preface* - when flying in offboard mode, a PX4 platform expects to recieve Attitude Commands (roll, pitch, yaw) in ENU frame. Keep this in mind.  

When using a PX4 platform with an external odometry estimator and offboard control there are 3 ways of going about it:

1) Calculate roll, pitch, yawrate and thrust commands onboard (in the **body frame**). Rotate roll and pitch body frame commands using yaw from magnetometer (e.g. ```mavros/imu/data```). No additional PX4 parameter setup needed, i.e. no need to read this.

2) Make PX4 think that the external pose estimate defines the ENU frame. To find out how to do that follow instructions in this document.

3) **TODO** - Maybe controlling the UAV by sending the Attidue Commands with body rates mitigates the issue, i.e. there is no need for
additional PX4 setup. 

## PX4 Parameter Setup

PX4 Parameters for enabling Optitrack fusion are set through the following command:
```bash
export UAV_NAMESPACE=red; roslaunch uav_ros_general px4_optitrack_params.launch
```
To find out more about what each parameter does please visit [PX4 Parameter Reference](http://docs.px4.io/master/en/advanced_config/parameter_reference.html).  
Parameters are determined according to this template [ekf2_indoor_vio.params](https://gitlab.com/voxl-public/flight-core-px4/px4-parameters/-/blob/master/helpers/ekf2_indoor_vio.params).  

Setting parameters does the following:

* Only enables the fusion of external position and yaw estimate. Disables GPS and magnetometer.
* Sets the external position as a primary altitude measurement.
* Sets the px4 EKF2 tuning specific for a given external estimator.

## ROS setup

The idea is to feed an external estimate to PX4 through Mavros interface. There are 2 available mavros topics:

* ```mavros/vision_pose/pose```  - geometry_msgs/PoseStamped
* ```mavros/odometry/out``` - nav_msgs/Odometry

To achieve this a following command is used:
```bash
export UAV_NAMESPACE=red; roslaunch uav_ros_general vision_pose_publisher.launch odometry:=vrpn_client/estimated_oometry pose_out:=mavros/vision_pose/pose
```
**NOTE** - when using ```mavros/vision_pose/pose``` it is important to set *EKF2_AID_MASK* to 24 and 
when using ```mavros/odometry/out``` it is important to set *EKF2_AID_MASK* 280 to signalize 
that we want the estimator to use velocity observations.

## Why not use ```mavros/odometry/out``` ?

A better solution would be to use ```mavros/odometry/out``` since it can provide additional velocity measurements from an 
external estimator. However, this doesn't seem to work on PX4 Firmware 1.11.3. A discussion is raised in the following links:

* [odometry-not-showing-up-in-px4-when-using-visual-interial-odometry-vio](https://discuss.px4.io/t/odometry-not-showing-up-in-px4-when-using-visual-interial-odometry-vio/18626/4)
* [mavlink::common::msg::ODOMETRY is not sending out for somereason](https://github.com/mavlink/mavros/issues/1208)
#include <uav_ros_general/ConstVelKF.h>
#include <uav_ros_general/VelocityEstimationParametersConfig.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <list>

int main(int argc, char** argv) 
{
  ros::init(argc, argv, "velocity_estimation_node");

  ros::NodeHandle nhPrivate("~");
  ros::NodeHandle nh;

  ConstVelKF kWrapperXPos("const_vel_x", nhPrivate);
  ConstVelKF kWrapperYPos("const_vel_y", nhPrivate);
  ConstVelKF kWrapperZPos("const_vel_z", nhPrivate);

  geometry_msgs::PoseStamped poseMsg;
  bool newMeasurementFlag = false;
  const auto poseCb = [&](const geometry_msgs::PoseStampedConstPtr& msg) {
    poseMsg = *msg;
    newMeasurementFlag = true;
  };
  
  auto odomPub = nh.advertise<nav_msgs::Odometry>("odometry", 1);
  static constexpr auto dt = 0.02;
  static constexpr auto throttle_time = 2.0;
  const auto timerCb = [&](const ros::TimerEvent & /* unused */) {
    
    kWrapperXPos.estimateState(dt, {poseMsg.pose.position.x}, newMeasurementFlag);
    kWrapperYPos.estimateState(dt, {poseMsg.pose.position.y}, newMeasurementFlag);
    kWrapperZPos.estimateState(dt, {poseMsg.pose.position.z}, newMeasurementFlag);

    nav_msgs::Odometry kalmanOdomMsg;
    kalmanOdomMsg.header.stamp = poseMsg.header.stamp;
    kalmanOdomMsg.header.frame_id = poseMsg.header.frame_id;
    kalmanOdomMsg.pose.pose.position.x = kWrapperXPos.getState()[0];
    kalmanOdomMsg.pose.pose.position.y = kWrapperYPos.getState()[0];
    kalmanOdomMsg.pose.pose.position.z = kWrapperZPos.getState()[0];
    kalmanOdomMsg.pose.pose.orientation = poseMsg.pose.orientation;
    kalmanOdomMsg.twist.twist.linear.x = kWrapperXPos.getState()[1];
    kalmanOdomMsg.twist.twist.linear.y = kWrapperYPos.getState()[1];
    kalmanOdomMsg.twist.twist.linear.z = - kWrapperZPos.getState()[1];
    odomPub.publish(kalmanOdomMsg);

    if (newMeasurementFlag) {
      newMeasurementFlag = false;
    }
  };
  
  auto poseSub = nh.subscribe<geometry_msgs::PoseStamped>("poseStamped", 1, poseCb);
  while (poseSub.getNumPublishers() == 0) {
    ROS_WARN_STREAM("Waiting for " << poseSub.getTopic() << " topic publisher.");
    ros::Duration(0.5).sleep();
  }

  ROS_INFO("Starting velocity_estimation_node...");
  auto kalmanTimer = nh.createTimer(ros::Duration(dt), timerCb);
  ros::spin();
}

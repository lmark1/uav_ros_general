#include <boost/array.hpp>
#include <uav_ros_lib/estimation/constant_acceleration_ukf.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <list>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "velocity_estimation_node");

  ros::NodeHandle nhPrivate("~");
  ros::NodeHandle nh;
  ConstantAccelerationUKF kWrapperXPos("const_acc_x", nhPrivate);
  ConstantAccelerationUKF kWrapperYPos("const_acc_y", nhPrivate);
  ConstantAccelerationUKF kWrapperZPos("const_acc_z", nhPrivate);
  
  geometry_msgs::PoseStamped poseMsg;
  bool newMeasurementFlag = false;
  const auto poseCb = [&](const geometry_msgs::PoseStampedConstPtr &msg) {
    poseMsg = *msg;
    newMeasurementFlag = true;
  };

  sensor_msgs::Imu myImuMsg;
  sensor_msgs::Imu oldImuMsg;
  const auto imuCb = [&](const sensor_msgs::ImuConstPtr &msg) {
    // Only do acceleration correction when acceleration measurement changes
    if (oldImuMsg.linear_acceleration.x != msg->linear_acceleration.x ||
        oldImuMsg.linear_acceleration.y != msg->linear_acceleration.y ||
        oldImuMsg.linear_acceleration.z != msg->linear_acceleration.z) {
      myImuMsg = *msg;
      auto quat = Eigen::Quaternionf(msg->orientation.w, msg->orientation.x,
                                     msg->orientation.y, msg->orientation.z);
      auto lin_acc = Eigen::Vector3f(msg->linear_acceleration.x,
                                     msg->linear_acceleration.y,
                                     msg->linear_acceleration.z);
      auto correct_acc = quat * lin_acc;
      myImuMsg.linear_acceleration.x = correct_acc.x();
      myImuMsg.linear_acceleration.y = correct_acc.y();
      myImuMsg.linear_acceleration.z = correct_acc.z();
    }

    oldImuMsg = *msg;
  };
  auto imuSub = nh.subscribe<sensor_msgs::Imu>("imu", 1, imuCb);

  auto odomPub = nh.advertise<nav_msgs::Odometry>("odometry", 1);
  auto imuPub = nh.advertise<sensor_msgs::Imu>("/imu_est", 1);
  static constexpr auto dt = 0.02;
  static constexpr auto throttle_time = 2.0;
  const auto timerCb = [&](const ros::TimerEvent & /* unused */) {
    kWrapperXPos.estimateState(
        dt, {poseMsg.pose.position.x, myImuMsg.linear_acceleration.x},
        newMeasurementFlag);
    kWrapperYPos.estimateState(
        dt, {poseMsg.pose.position.y, myImuMsg.linear_acceleration.y},
        newMeasurementFlag);
    kWrapperZPos.estimateState(
        dt,
        {poseMsg.pose.position.z, (myImuMsg.linear_acceleration.z - 9.80689)},
        newMeasurementFlag);

    nav_msgs::Odometry kalmanOdomMsg;
    kalmanOdomMsg.header.stamp = poseMsg.header.stamp;
    kalmanOdomMsg.header.frame_id = poseMsg.header.frame_id;
    kalmanOdomMsg.pose.pose.position.x = kWrapperXPos.getState()[0];
    kalmanOdomMsg.pose.pose.position.y = kWrapperYPos.getState()[0];
    kalmanOdomMsg.pose.pose.position.z = kWrapperZPos.getState()[0];
    kalmanOdomMsg.pose.pose.orientation = poseMsg.pose.orientation;
    kalmanOdomMsg.twist.twist.linear.x = kWrapperXPos.getState()[1];
    kalmanOdomMsg.twist.twist.linear.y = kWrapperYPos.getState()[1];
    kalmanOdomMsg.twist.twist.linear.z = -kWrapperZPos.getState()[1];
    kalmanOdomMsg.pose.covariance[0] = kWrapperXPos.getStateCovariance()(0, 0);
    kalmanOdomMsg.pose.covariance[1] = kWrapperYPos.getStateCovariance()(0, 0);
    kalmanOdomMsg.pose.covariance[2] = kWrapperZPos.getStateCovariance()(0, 0);
    kalmanOdomMsg.pose.covariance[3] = kWrapperXPos.getStateCovariance()(1, 1);
    kalmanOdomMsg.pose.covariance[4] = kWrapperYPos.getStateCovariance()(1, 1);
    kalmanOdomMsg.pose.covariance[5] = kWrapperZPos.getStateCovariance()(1, 1);
    kalmanOdomMsg.pose.covariance[6] = kWrapperXPos.getStateCovariance()(2, 2);
    kalmanOdomMsg.pose.covariance[7] = kWrapperYPos.getStateCovariance()(2, 2);
    kalmanOdomMsg.pose.covariance[8] = kWrapperZPos.getStateCovariance()(2, 2);
    kalmanOdomMsg.pose.covariance[9] = kWrapperXPos._nis;
    kalmanOdomMsg.pose.covariance[10] = kWrapperYPos._nis;
    kalmanOdomMsg.pose.covariance[11] = kWrapperZPos._nis;
    odomPub.publish(kalmanOdomMsg);

    sensor_msgs::Imu estImuMsg;
    estImuMsg.header.frame_id = myImuMsg.header.frame_id;
    estImuMsg.header.stamp = myImuMsg.header.stamp;
    estImuMsg.linear_acceleration.x = kWrapperXPos.getState()[2];
    estImuMsg.linear_acceleration.y = kWrapperYPos.getState()[2];
    estImuMsg.linear_acceleration.z = kWrapperZPos.getState()[2];
    estImuMsg.angular_velocity.x = myImuMsg.linear_acceleration.x;
    estImuMsg.angular_velocity.y = myImuMsg.linear_acceleration.y;
    estImuMsg.angular_velocity.z = myImuMsg.linear_acceleration.z;
    imuPub.publish(estImuMsg);

    if (newMeasurementFlag) {
      newMeasurementFlag = false;
    }
  };

  auto poseSub =
      nh.subscribe<geometry_msgs::PoseStamped>("poseStamped", 1, poseCb);
  while (poseSub.getNumPublishers() == 0) {
    ROS_WARN_STREAM("Waiting for " << poseSub.getTopic()
                                   << " topic publisher.");
    ros::Duration(0.5).sleep();
  }

  ROS_INFO("Starting velocity_estimation_node...");
  auto kalmanTimer = nh.createTimer(ros::Duration(dt), timerCb);
  ros::spin();
}
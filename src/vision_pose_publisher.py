#!/usr/bin/env python

import rospy
import tf
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped, Quaternion, TransformStamped, Vector3Stamped
from nav_msgs.msg import Odometry

def rotate_and_publish(pose_stamped):
  # Transform position
  position = Vector3Stamped()
  position.vector.x = pose_stamped.pose.position.x
  position.vector.y = pose_stamped.pose.position.y
  position.vector.z = pose_stamped.pose.position.z
  position_frd = tf2_geometry_msgs.do_transform_vector3(position, transform_frd)

  # Transform rotation
  quaternion_xyz = Vector3Stamped()
  quaternion_xyz.vector.x = pose_stamped.pose.orientation.x
  quaternion_xyz.vector.y = pose_stamped.pose.orientation.y
  quaternion_xyz.vector.z = pose_stamped.pose.orientation.z
  quaternion_xyz_frd = tf2_geometry_msgs.do_transform_vector3(quaternion_xyz, transform_frd)

  # Construct and publish FRD position
  pose_stamped_frd = PoseStamped()
  pose_stamped_frd.header = pose_stamped.header
  pose_stamped_frd.pose.position.x = position_frd.vector.x
  pose_stamped_frd.pose.position.y = position_frd.vector.y
  pose_stamped_frd.pose.position.z = position_frd.vector.z
  pose_stamped_frd.pose.orientation.x = quaternion_xyz_frd.vector.x
  pose_stamped_frd.pose.orientation.y = quaternion_xyz_frd.vector.y
  pose_stamped_frd.pose.orientation.z = quaternion_xyz_frd.vector.z
  pose_stamped_frd.pose.orientation.w = pose_stamped.pose.orientation.w # WTF
  vision_pose_pub.publish(pose_stamped_frd)
  return pose_stamped_frd

def pose_cb(msg):
  rotate_and_publish(msg)

def odom_cb(msg):
  pose_stamped = PoseStamped()
  pose_stamped.header = msg.header
  pose_stamped.pose = msg.pose.pose
  pose_stamped_frd = rotate_and_publish(pose_stamped)

  odom_frd = msg
  odom_frd.header.frame_id = "odom"
  odom_frd.child_frame_id = "base_link"
  odom_pub.publish(odom_frd)


if __name__ == '__main__':
  rospy.init_node("vision_pose_publisher")
  vision_pose_pub = rospy.Publisher("pose_out", PoseStamped, queue_size=1)
  odom_pub = rospy.Publisher("odometry_out", Odometry, queue_size=1)
  transform_frd = TransformStamped() # Transform the obtained pose / odometry to F(forward)-R(right)-D(down) frame

  transform_frd.transform.rotation.x = rospy.get_param("~vision_pose/rotation/x")
  transform_frd.transform.rotation.y = rospy.get_param("~vision_pose/rotation/y")
  transform_frd.transform.rotation.z = rospy.get_param("~vision_pose/rotation/z")
  transform_frd.transform.rotation.w = rospy.get_param("~vision_pose/rotation/w")
  
  rospy.Subscriber("pose", PoseStamped, pose_cb)
  rospy.Subscriber("odometry", Odometry, odom_cb)

  rospy.spin()
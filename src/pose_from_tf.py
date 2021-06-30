#!/usr/bin/env python

from geometry_msgs.msg import PoseStamped, TransformStamped
import rospy
import tf2_ros

if __name__ == '__main__':
    rospy.init_node('pose_from_tf')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    pose_pub = rospy.Publisher("slam_pose", PoseStamped, queue_size=10)
    transform_pub = rospy.Publisher("slam_transform", TransformStamped, queue_size=10)

    tracking_frame = rospy.get_param("~tracking_frame")
    map_frame = rospy.get_param("~map_frame")
    publish_rate = rospy.get_param("~publish_rate")

    rate = rospy.Rate(publish_rate)
    while not rospy.is_shutdown():
        try:
            # The tracking frame and the laser frame are swapped. Therefore lookup laser_frame.
            trans_msg = tfBuffer.lookup_transform(map_frame, tracking_frame, rospy.Time(0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as ex:
            rospy.logwarn(ex)
            rate.sleep()
            continue
        
        # Publish pose message
        pose_msg = PoseStamped()
        pose_msg.header = trans_msg.header
        pose_msg.pose.position.x = trans_msg.transform.translation.x
        pose_msg.pose.position.y = trans_msg.transform.translation.y
        pose_msg.pose.position.z = trans_msg.transform.translation.z
        pose_msg.pose.orientation.x = trans_msg.transform.rotation.x
        pose_msg.pose.orientation.y = trans_msg.transform.rotation.y
        pose_msg.pose.orientation.z = trans_msg.transform.rotation.z
        pose_msg.pose.orientation.w = trans_msg.transform.rotation.w
        pose_pub.publish(pose_msg)
        transform_pub.publish(trans_msg)
        rate.sleep()


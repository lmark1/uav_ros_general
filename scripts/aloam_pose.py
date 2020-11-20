#!/usr/bin/env python

import geometry_msgs.msg
import math
import nav_msgs
import numpy as np
import rospy
import tf2_ros




if __name__ == '__main__':
    rospy.init_node('aloam_pose')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    publisher = rospy.Publisher("aloam/pose", geometry_msgs.msg.PoseStamped, queue_size=100)

    tracking_frame = rospy.get_param("tracking_frame")
    laser_frame = rospy.get_param("laser_frame")
    map_frame = rospy.get_param("map_frame")

    rate = rospy.Rate(50)       # Todo what rate to use? 50 Hz may be too much for loam
    while not rospy.is_shutdown():
        try:
            # The tracking frame and the laser frame are swapped. Therefore lookup laser_frame.
            trans = tfBuffer.lookup_transform(map_frame, laser_frame, rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as ex:
            rospy.logwarn(ex)
            rate.sleep()
            continue

        msg = geometry_msgs.msg.PoseStamped()

        msg.pose.position.x = trans.transform.translation.x
        msg.pose.position.y = trans.transform.translation.y
        msg.pose.position.z = trans.transform.translation.z

        msg.pose.orientation.x = trans.transform.rotation.x
        msg.pose.orientation.y = trans.transform.rotation.y
        msg.pose.orientation.z = trans.transform.rotation.z
        msg.pose.orientation.w = trans.transform.rotation.w

        publisher.publish(msg)
        rate.sleep()


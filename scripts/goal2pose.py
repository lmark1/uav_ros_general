#!/usr/bin/env python

import rospy 
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64

pose_pub = rospy.Publisher("/erl_uav/command/pos_ref", PoseStamped, queue_size=1)
height_ref = 3

def height_cb(data):
    global height_ref
    height_ref = data.data

def goal_cb(data):
    global pose_pub
    newMsg = PoseStamped()
    
    newMsg.pose.position.x = data.pose.position.x
    newMsg.pose.position.y = data.pose.position.y
    newMsg.pose.position.z = height_ref
    
    newMsg.pose.orientation.x = data.pose.orientation.x
    newMsg.pose.orientation.y = data.pose.orientation.y
    newMsg.pose.orientation.z = data.pose.orientation.z
    newMsg.pose.orientation.w = data.pose.orientation.w

    pose_pub.publish(newMsg)

if __name__ == '__main__':
    rospy.init_node("goal2pose_node")

    rospy.Subscriber("/move_base_simple/goal", PoseStamped, goal_cb)
    rospy.Subscriber("/erl_uav/height_ref", Float64, height_cb)
    rospy.spin()

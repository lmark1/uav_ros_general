#!/usr/bin/env python
import rospy
from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg import Pose, Point, PoseStamped, Quaternion, Transform
from tf import transformations
from math import pi, sqrt, sin, cos
import numpy as np

T_local_ecef = Transform()
t_init = False
pub = rospy.Publisher('erl_uav/goal_pose', PoseStamped, queue_size=10)
A_EARTH = 6378137.0;
flattening = 1.0/298.257223563;
NAV_E2 = (2.0-flattening)*flattening;
deg2rad = pi/180.0;

def callback_ecef(data):
    print("global2pose: ECEF INIT")
    global T_local_ecef
    T_local_ecef = np.dot(transformations.translation_matrix([data.translation.x, data.translation.y, data.translation.z]), transformations.quaternion_matrix([data.rotation.x, data.rotation.y, data.rotation.z, data.rotation.w]))
    global t_init
    t_init = True


def callback_goal(data):
    print("gloabl2pose: sp_goal")
    if t_init:
        print("Inside init")
        lat = data.latitude
        lon = data.longitude
        alt = data.altitude

        slat = sin(lat*deg2rad);
        clat = cos(lat*deg2rad);
        r_n = A_EARTH/sqrt(1.0 - NAV_E2*slat*slat);
        point = Point()
        x = (r_n + alt)*clat*cos(lon*deg2rad)
        y = (r_n + alt)*clat*sin(lon*deg2rad)
        z = (r_n*(1.0 - NAV_E2) + alt)*slat
        point_transformed = np.dot(T_local_ecef, np.array([x, y, z, 1.0]))
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = rospy.Time.now()
	
        goal.pose.position.x = point_transformed[0]
        goal.pose.position.y = point_transformed[1]
        goal.pose.position.z = point_transformed[2]
	
	quaternion = transformations.quaternion_from_euler(0, 0, 0)
        goal.pose.orientation.x = quaternion[0]
        goal.pose.orientation.y = quaternion[1]
        goal.pose.orientation.z = quaternion[2]
        goal.pose.orientation.w = quaternion[3]
        pub.publish(goal)


def transform_goal():
    rospy.init_node('transform_goal', anonymous=True)
    rospy.Subscriber("ecef_to_local", Transform, callback_ecef)
    rospy.Subscriber("erl_uav/global_setpoint_goal", GeoPoint, callback_goal)
    rospy.spin()

if __name__ == '__main__':
    transform_goal()


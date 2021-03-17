#!/usr/bin/env python

import roslib
import rospy 
import math 
import tf
from geometry_msgs.msg import TransformStamped, Transform, Pose, PoseStamped
from mavros_msgs.msg import GlobalPositionTarget
from nav_msgs.msg import Odometry
from std_msgs.msg import Header, Float64
from tf import transformations as t
import numpy as np 
import pyproj
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import Imu

cb_check = False
INVERSE_TRANSFORM = None
TOTAL_TRANSFORM = None
pub_global = rospy.Publisher("mavros/setpoint_raw/global", GlobalPositionTarget, queue_size=1)
pub_local = rospy.Publisher("uav/cartographer/pose", PoseStamped, queue_size=1)
yaw_ref = 0

ecef = pyproj.Proj(proj='geocent', ellps='WGS84', datum='WGS84')
lla = pyproj.Proj(proj='latlong', ellps='WGS84', datum='WGS84')
rotation_list = [0, 0, 0, 1]

def imu_cb(data):
    rotation_list[0] = data.orientation.x
    rotation_list[1] = data.orientation.y
    rotation_list[2] = data.orientation.z
    rotation_list[3] = data.orientation.w

def command_cb(data):

    if INVERSE_TRANSFORM is None:
        print("Waiting for local2ecef transform")
        return

    # Make a position vector
    pos_ref = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z, 1])
    pos_ref.shape = (4, 1)
    
    # Transformed pose
    pos_ecef = np.dot(INVERSE_TRANSFORM, pos_ref)

    x = pos_ecef[0]
    y = pos_ecef[1]
    z = pos_ecef[2]
    lon, lat, alt = pyproj.transform(ecef, lla, x, y, z, radians=False)
    #print("Publishing global waypoint: Lon = {}, lat = {}, alt = {}".format(lon, lat, alt))

    # Publish to UAV
    newMsg = GlobalPositionTarget()
    newMsg.type_mask = 2552
    newMsg.coordinate_frame = 5
    newMsg.latitude = lat
    newMsg.longitude = lon
    newMsg.altitude = alt - 48.2413

    # Calculate yaw
    #rot_list = [data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w]
    #(roll, pitch, yaw_ref) = euler_from_quaternion(rot_list)
    
    global yaw_ref
    newMsg.yaw = yaw_ref
    pub_global.publish(newMsg)
    
def ecef2local_cb(data):
    global INVERSE_TRANSFORM
    global cb_check

    cb_check = True
    translation = [data.translation.x, data.translation.y, data.translation.z]
    quaternion = [data.rotation.x, data.rotation.y, data.rotation.z, data.rotation.w]
    
    print("Hello from inverse_transform callback")
    transform_matrix = t.concatenate_matrices(
        t.translation_matrix(translation), t.quaternion_matrix(quaternion))
    INVERSE_TRANSFORM = t.inverse_matrix(transform_matrix)
    print("Inverse transform matrix: ", INVERSE_TRANSFORM)

def yaw_cb(data):
    global yaw_ref
    yaw_ref = data.data

if __name__ == '__main__':
    global cb_check
    cb_check = False

    rospy.init_node("pose2WG84_node")
    listener = tf.TransformListener()
    rospy.Subscriber("ecef_to_local", Transform, ecef2local_cb)
    rospy.Subscriber("erl_uav/command/pos_ref", PoseStamped, command_cb)
    rospy.Subscriber("mavros/imu/data", Imu, imu_cb)
    rospy.Subscriber("erl_uav/yaw_ref", Float64, yaw_cb)
    parent_frame = rospy.get_param("~parent_frame")
    child_frame = rospy.get_param("~child_frame")   
    rate = rospy.Rate(50)

    while not rospy.is_shutdown():
        rate.sleep()

        try:
            frame1 = parent_frame
            frame2 = child_frame
            (trans, rot) = listener.lookupTransform(frame1, frame2, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("Wait for {} to {} transform".format(frame1, frame2))
            continue

        newMsg = PoseStamped()
	newMsg.header.stamp = rospy.get_rostime()
	newMsg.header.frame_id = parent_frame
        newMsg.pose.position.x = trans[0]
        newMsg.pose.position.y = trans[1]
        newMsg.pose.position.z = trans[2]
	
        newMsg.pose.orientation.x = rot[0] #rotation_list[0]
        newMsg.pose.orientation.y = rot[1] #rotation_list[1]
        newMsg.pose.orientation.z = rot[2] #rotation_list[2]
        newMsg.pose.orientation.w = rot[3] #rotation_list[3]
        pub_local.publish(newMsg)

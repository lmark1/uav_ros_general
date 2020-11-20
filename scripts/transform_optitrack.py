#!/usr/bin/env python
import rospy

from geometry_msgs.msg import  PoseStamped
import math
import numpy as np
from math import sqrt, sin, cos, pi, atan, floor, atan2

import tf2_ros
import tf
from scipy.spatial.transform import Rotation

import message_filters

class transformOptitrack():
    def __init__(self):

        self.new_target_data = False
        self.new_zed_data = False

        self.target_data = PoseStamped()
        self.zed_data = PoseStamped()

        self.target_msg = PoseStamped()
        self.target_msg.header.frame_id = "world2"

        self.zed_msg = PoseStamped()
        self.zed_msg.header.frame_id = "world2"

        # Create subscribers
        rospy.Subscriber('/vrpn_client_node/target/pose', PoseStamped, self.target_pose_cb)
        rospy.Subscriber('/vrpn_client_node/zed/pose', PoseStamped, self.zed_pose_cb)

        # Create publishers
        self.target_world_pub = rospy.Publisher('/vrpn_client_node/target/world/pose', PoseStamped, queue_size = 1)
        self.zed_world_pub = rospy.Publisher('/vrpn_client_node/zed/world/pose', PoseStamped, queue_size = 1)

        # euler = [1.5707963268, 0.0, 0.0]
        euler = [math.pi/2, 0.0, math.pi/2]
        position = [0.0 , 0.0, 0.0]

        self.Tuavr_optitrack = self.getRotationTranslationMatrix(euler, position)

    """ Callbacks """
    def target_pose_cb(self, data):
        self.new_target_data = True
        self.target_data = data

    def zed_pose_cb(self, data):
        self.new_zed_data = True
        self.zed_data = data

    def quaternion2euler(self, quaternion):

        euler = [0.0, 0.0, 0.0]

        euler[0] = math.atan2(2 * (quaternion[0] * quaternion[1] + quaternion[2] * quaternion[3]), 1 - 2 * (quaternion[1] * quaternion[1] + quaternion[2] * quaternion[2]))

        euler[1] = math.asin(2 * (quaternion[0] * quaternion[2] - quaternion[3] * quaternion[1]))

        euler[2] = math.atan2(2 * (quaternion[0]*quaternion[3] + quaternion[1]*quaternion[2]), 1 - 2 * (quaternion[2]*quaternion[2] + quaternion[3] * quaternion[3]))

        return euler

    def euler2quaternion(self, euler):
        quaternion = [1.0, 0.0, 0.0, 0.0]

        cy = cos(euler[2] * 0.5)
        sy = sin(euler[2] * 0.5)
        cr = cos(euler[0] * 0.5)
        sr = sin(euler[0] * 0.5)
        cp = cos(euler[1] * 0.5)
        sp = sin(euler[1] * 0.5)

        quaternion[0] = cy * cr * cp + sy * sr * sp; #w
        quaternion[1] = cy * sr * cp - sy * cr * sp; #x
        quaternion[2] = cy * cr * sp + sy * sr * cp; #y
        quaternion[3] = sy * cr * cp - cy * sr * sp; #z

        return quaternion

    def getRotationTranslationMatrix(self, orientationEuler, position):
        """
        Rotation & Translation Matrix from UAV to world coordinates.
        """
        x = orientationEuler[0]
        y = orientationEuler[1]
        z = orientationEuler[2]

        r11 = math.cos(y)*math.cos(z)

        r12 = math.cos(z)*math.sin(x)*math.sin(y) - math.cos(x)*math.sin(z)

        r13 = math.sin(x)*math.sin(z) + math.cos(x)*math.cos(z)*math.sin(y)

        r21 = math.cos(y)*math.sin(z)

        r22 = math.cos(x)*math.cos(z) + math.sin(x)*math.sin(y)*math.sin(z)

        r23 = math.cos(x)*math.sin(y)*math.sin(z) - math.cos(z)*math.sin(x)

        r31 = -math.sin(y)

        r32 = math.cos(y)*math.sin(x)

        r33 = math.cos(x)*math.cos(y)

        t1 = position[0]
        t2 = position[1]
        t3 = position[2]

        rotationTranslationMatrix = np.zeros((4, 4))

        rotationTranslationMatrix[0, 0] = r11
        rotationTranslationMatrix[0, 1] = r12
        rotationTranslationMatrix[0, 2] = r13
        rotationTranslationMatrix[0, 3] = t1

        rotationTranslationMatrix[1, 0] = r21
        rotationTranslationMatrix[1, 1] = r22
        rotationTranslationMatrix[1, 2] = r23
        rotationTranslationMatrix[1, 3] = t2

        rotationTranslationMatrix[2, 0] = r31
        rotationTranslationMatrix[2, 1] = r32
        rotationTranslationMatrix[2, 2] = r33
        rotationTranslationMatrix[2, 3] = t3

        rotationTranslationMatrix[3, 0] = 0.0
        rotationTranslationMatrix[3, 1] = 0.0
        rotationTranslationMatrix[3, 2] = 0.0
        rotationTranslationMatrix[3, 3] = 1.0

        return rotationTranslationMatrix

    def transformTarget(self):

        # Target
        #quaternion = [self.target_data.pose.orientation.w, self.target_data.pose.orientation.x, self.target_data.pose.orientation.y, self.target_data.pose.orientation.z]
        # euler = self.quaternion2euler(quaternion)

        # other way to calc euler
        rot = Rotation.from_quat([self.target_data.pose.orientation.x, self.target_data.pose.orientation.y, self.target_data.pose.orientation.z, self.target_data.pose.orientation.w])
        euler2 = rot.as_euler('xyz', degrees=False)

        position = [self.target_data.pose.position.x, self.target_data.pose.position.y, self.target_data.pose.position.z]

        target_optitrack = self.getRotationTranslationMatrix(euler2, position)

        target_uavr = np.dot(self.Tuavr_optitrack, target_optitrack)

        self.target_msg.header.stamp = rospy.Time.now()
        self.target_msg.pose.position.x = target_uavr[0,3]
        self.target_msg.pose.position.y = target_uavr[1,3]
        self.target_msg.pose.position.z = target_uavr[2,3]

        q = self.Tuavr_optitrack.dot([self.target_data.pose.orientation.x, self.target_data.pose.orientation.y, self.target_data.pose.orientation.z, self.target_data.pose.orientation.w])

        self.target_msg.pose.orientation.x = q[0]
        self.target_msg.pose.orientation.y = q[1]
        self.target_msg.pose.orientation.z = q[2]
        self.target_msg.pose.orientation.w = q[3]

        self.target_world_pub.publish(self.target_msg)

    def transfromZED(self):

        # ZED
        # quaternion = [self.zed_data.pose.orientation.w, self.zed_data.pose.orientation.x, self.zed_data.pose.orientation.y, self.zed_data.pose.orientation.z]
        # euler = self.quaternion2euler(quaternion)

        # other way to calc euler
        rot = Rotation.from_quat([self.zed_data.pose.orientation.x, self.zed_data.pose.orientation.y, self.zed_data.pose.orientation.z, self.zed_data.pose.orientation.w])
        euler2 = rot.as_euler('xyz', degrees=False)

        position = [self.zed_data.pose.position.x, self.zed_data.pose.position.y, self.zed_data.pose.position.z]

        zed_optitrack = self.getRotationTranslationMatrix(euler2, position)

        zed_uavr = np.dot(self.Tuavr_optitrack, zed_optitrack)

        self.zed_msg.header.stamp = rospy.Time.now()
        self.zed_msg.pose.position.x = zed_uavr[0,3]
        self.zed_msg.pose.position.y = zed_uavr[1,3]
        self.zed_msg.pose.position.z = zed_uavr[2,3]

        q = self.Tuavr_optitrack.dot([self.zed_data.pose.orientation.x, self.zed_data.pose.orientation.y, self.zed_data.pose.orientation.z, self.zed_data.pose.orientation.w])

        self.zed_msg.pose.orientation.x = q[0]
        self.zed_msg.pose.orientation.y = q[1]
        self.zed_msg.pose.orientation.z = q[2]
        self.zed_msg.pose.orientation.w = q[3]

        self.zed_world_pub.publish(self.zed_msg)

    def run(self, rate):
        r = rospy.Rate(rate)

        while not rospy.is_shutdown():

            if self.new_target_data:
                self.transformTarget()
                self.new_target_data = False

            if self.new_zed_data:
                self.transfromZED()
                self.new_zed_data = False

            #r.sleep()

if __name__ == '__main__':
    rospy.init_node('transformOptitrack')

    t = transformOptitrack()
    t.run(15)

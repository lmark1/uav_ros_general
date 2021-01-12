#!/usr/bin/python

import roslib
import rospy
import math
from geometry_msgs.msg import Twist, PoseStamped, TwistStamped
from sensor_msgs.msg import Joy, Imu, NavSatFix
from std_msgs.msg import Empty
from mavros_msgs.msg import GlobalPositionTarget

class UavJoyReference:

    def __init__(self):
        # Publisher to ardrone cmd_vel topic, can be run in namespace 48.213
        self.global_position_target_pub = rospy.Publisher("/mavros/setpoint_raw/global", 
            GlobalPositionTarget, queue_size=1)

        # Initialize joy and cmd_vel variables
        self.joy_data = Joy()
        self.command_velocity = Twist() # Publishing to real cmd_vel
        self.override_control = 0
        self.current_reference = GlobalPositionTarget()
        self.current_reference.coordinate_frame=GlobalPositionTarget.FRAME_GLOBAL_INT
        self.current_reference.type_mask=2552

        self.max_vel_x = rospy.get_param("~max_vel_x", 0.5)
        self.max_vel_y = rospy.get_param("~max_vel_y", 0.5)
        self.max_vel_z = rospy.get_param("~max_vel_z", 0.5)
        self.max_vel_yaw = rospy.get_param("~max_vel_yaw", 0.1)
        self.rate = rospy.get_param("~rate", 20)
        self.gps_factor = 0.00001

        self.global_yaw = 0.0
        self.get_current_global_reference_flag = 0
        self.first_global_reference_flag = 0


        # Subscriber to joystick topic
        rospy.Subscriber("/manual_control/joy", Joy, self.joyCallback, queue_size=1)
        # subscriber na trenutnu globalnu poziciju
        rospy.Subscriber("/mavros/global_position/global", NavSatFix, self.currentGlobalPositionCallback, queue_size=1)
        # subscriber to imu for yaw 
        rospy.Subscriber("/mavros/imu/data", Imu, self.imuCallback, queue_size=1)

    def run(self):
        r = rospy.Rate(int(20))

        while not rospy.is_shutdown():
            if self.override_control == 1 and self.first_global_reference_flag == 1:
                self.current_reference.longitude = self.current_reference.longitude + \
                    (self.command_velocity.linear.x*self.max_vel_x/float(self.rate))*self.gps_factor
                self.current_reference.latitude = self.current_reference.latitude + \
                    (self.command_velocity.linear.y*self.max_vel_y/float(self.rate))*self.gps_factor
                self.current_reference.altitude = self.current_reference.altitude + \
                    (self.command_velocity.linear.z*self.max_vel_z/float(self.rate))
                self.global_yaw = self.global_yaw + \
                    (self.command_velocity.angular.z*self.max_vel_yaw/float(self.rate))
                self.current_reference.yaw = math.atan2(math.sin(self.global_yaw), math.cos(self.global_yaw))
                #print self.global_yaw, self.current_reference.yaw

                self.global_position_target_pub.publish(self.current_reference)
            
            r.sleep()

    def joyCallback(self, data):
        # Assign data to joy variable
        self.joyData = data

        #print self.joyData

        # Setting joy values to be command values for bebop
        vx = self.joyData.axes[3]
        vy = -self.joyData.axes[2]
        self.command_velocity.linear.x = math.cos(self.global_yaw)*vx + math.sin(self.global_yaw)*vy
        self.command_velocity.linear.y = math.sin(self.global_yaw)*vx - math.cos(self.global_yaw)*vy
        self.command_velocity.linear.z = self.joyData.axes[1]
        self.command_velocity.angular.z = self.joyData.axes[0]

        self.override_control = self.joyData.buttons[6]
        self.get_current_global_reference_flag = self.joyData.buttons[7]

    def currentGlobalPositionCallback(self, data):
        if self.get_current_global_reference_flag:
            self.current_reference.latitude = data.latitude
            self.current_reference.longitude = data.longitude
            self.current_reference.altitude = data.altitude-48.213
            self.first_global_reference_flag = 1
            #self.current_reference.yaw = 0.0
            print "Setting reference to current position."

    def imuCallback(self, data):
        q0 = data.orientation.w
        q1 = data.orientation.x
        q2 = data.orientation.y
        q3 = data.orientation.z

        if self.get_current_global_reference_flag:
            self.global_yaw = math.atan2(2.0*(q0*q3 + q1*q2), 1.0-2.0*(q2*q2 + q3*q3))
            #self.current_reference.yaw = self.global_yaw
            print "Setting yaw reference."


if __name__ == "__main__":
    rospy.init_node("uav_joy_reference")
    joyControl = UavJoyReference()
    joyControl.run()

# Takeoff: R2
# Land: L2
# Reset: Start
# Take control: R1 - [5]
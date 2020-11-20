#!/usr/bin/env python

import rospy
from sensor_msgs.msg import NavSatFix
from geographic_msgs.msg import GeoPoint
from math import sqrt
from std_msgs.msg import Float64

class WaypointControl:

    def __init__(self, waypoints, tol):
        self.waypoints = waypoints
        self.tol = tol

        self.wp_count = len(self.waypoints)
        self.wp_index = 0

        self.waypoint_published = False

        self.lat = 0
        self.lon = 0
        self.alt = 0
        self.global_cb_flag = False

        rospy.Subscriber("/mavros/global_position/global", NavSatFix, self.global_cb)
        self.glob_pos_pub = rospy.Publisher("erl_uav/global_setpoint_goal", GeoPoint, queue_size=1)
        self.yaw_pub = rospy.Publisher("erl_uav/yaw_ref", Float64, queue_size=1)

    def global_cb(self, data):
        self.global_cb_flag = True
        self.curr_lat = data.latitude
        self.curr_lon = data.longitude
        self.curr_alt = data.altitude

    def follow_waypoints(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and not self.global_cb_flag:
            print("Waiting for global position callback")
            rate.sleep()
     	
        print("WP published sleeping - 5s")
	rospy.sleep(5)
        print("Waypoint control: waypoint following started")
        print(self.waypoints)
        while self.wp_index < self.wp_count and not rospy.is_shutdown():
            rate.sleep()
            wp = self.waypoints[self.wp_index]
            
            wp_lat = wp[0]
            wp_lon = wp[1]
            wp_alt = wp[2]
            wp_yaw = wp[3]

            # Ignore altitude
            
            wp_norm = sqrt((wp_lat - self.curr_lat)**2 + (wp_lon - self.curr_lon)**2)
            #print("Distance to wp {} : {}, tol: {}".format(self.wp_index, wp_norm, self.tol))
                
            if wp_norm < float(self.tol):
                print("Waypoint {} reched.".format(self.wp_index))
                self.wp_index += 1
                self.waypoint_published = False
                raw_input("Press any key to continue")
            else:
                #print("Distance to wp {} : {}, tol: {}".format(self.wp_index, wp_norm, self.tol))
                yaw_msg = Float64()
                yaw_msg.data = wp_yaw
                self.yaw_pub.publish(yaw_msg)

                if not self.waypoint_published:
                    print("Publishing waypoint")
                    self.waypoint_published = True
                    newMsg = GeoPoint()
                    newMsg.latitude = wp_lat
                    newMsg.longitude = wp_lon
                    newMsg.altitude = wp_alt
                    self.glob_pos_pub.publish(newMsg)

if __name__ == '__main__':
    rospy.init_node("wp_publisher_node")
    waypoint_list = rospy.get_param("waypoints")
    wp_tol = rospy.get_param("tol")
    wp_ctrl = WaypointControl(waypoint_list, wp_tol)
    wp_ctrl.follow_waypoints()


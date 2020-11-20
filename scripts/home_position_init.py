#!/usr/bin/env python

import rospy 
from mavros_msgs.msg import HomePosition
from sensor_msgs.msg import NavSatFix

class HomePositionInit:
  def __init__(self):  
    self.home_pub = rospy.Publisher("mavros/global_position/home", HomePosition, queue_size=1, latch=True)
    rospy.sleep(1)
    self.gps_sub = rospy.Subscriber("mavros/global_position/global", NavSatFix, self.global_cb)
    self.is_published = False
    self.set_fixed = rospy.get_param('~set_fixed_home', False)
    self.home_lat = rospy.get_param('~home_lat', 0)
    self.home_lon = rospy.get_param('~home_lon', 0)
    self.home_alt = rospy.get_param('~home_alt', 0)

    if self.set_fixed:
      print("Fixed home position is going to be set: ", self.home_lat, self.home_lon, self.home_alt)
    else:
      print("HomePosition is set to current GPS position")

  def global_cb(self, data):
    if self.is_published:
      return

    print("HomePositionInit: Global position recieved")
    homeMsg = HomePosition()
    homeMsg.header.stamp = rospy.Time.now()
    
    if self.set_fixed:
      homeMsg.geo.latitude = self.home_lat
      homeMsg.geo.longitude = self.home_lon
      homeMsg.geo.altitude = self.home_alt
    else:
      homeMsg.geo.latitude = data.latitude
      homeMsg.geo.longitude = data.longitude
      homeMsg.geo.altitude = data.altitude
      
    self.home_pub.publish(homeMsg)
    self.is_published = True

  def isHomePublished(self):
    return self.is_published


if __name__ == '__main__':
  rospy.init_node("home_init_node")
  hpi = HomePositionInit()
  while not rospy.is_shutdown() and not hpi.isHomePublished():
    print("home_init_node: Try initializing home position")
    rospy.sleep(0.01)
  print("home_init_node: Spinning for latch")
  rospy.spin()

#!/usr/bin/env python

import rospy
import sensor_msgs.point_cloud2 as pc2
import tf2_ros
import tf2_py as tf2
from sensor_msgs.msg import PointCloud2
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

class Transformer:
  """Transforms pointcloud2 from laser_frame to tracking_frame.
    Requires the tf2-sensor-msgs package
  """

  def __init__(self):
    self.tf_buffer = tf2_ros.Buffer()
    self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    self.subscriber = rospy.Subscriber("points_in", PointCloud2, self.callback)
    self.publisher = rospy.Publisher("points_out", PointCloud2, queue_size=100)

    self.tracking_frame = rospy.get_param("tracking_frame")

  def callback(self, msg):
    try:
      transform = self.tf_buffer.lookup_transform(self.tracking_frame, msg.header.frame_id,
                                          msg.header.stamp, rospy.Duration(2))
    except tf2.LookupException as ex:
        rospy.logwarn(ex)
        return
    except tf2.ExtrapolationException as ex:
        rospy.logwarn(ex)
        return
    cloud_out = do_transform_cloud(msg,transform)
    self.publisher.publish(cloud_out)


if __name__ == '__main__':
  rospy.init_node('cloud_transformer')
  transformer = Transformer()
  rospy.spin()


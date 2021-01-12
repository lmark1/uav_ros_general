#!/usr/bin/python

import math
import rospy
from geometry_msgs.msg import Quaternion, PoseStamped, Transform, Twist
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint
import tf
import tf_conversions


class StepPublisher:
    """
    Helper node for publishing step reference signals.
    
    Usage: run the node and publish the desired step on the step_reference topic.
    
           The node will add the desired step to the last received carrot/pose message
           and publish the result on the position_hold/trajectory topic
    """

    def __init__(self):
        rospy.Subscriber('carrot/pose', PoseStamped, self.carrotCb)
        rospy.Subscriber('step_reference', PoseStamped, self.stepCb)
        self.publisher = rospy.Publisher('position_hold/trajectory', MultiDOFJointTrajectoryPoint, queue_size=10)
        self.carrot_received = False

    def carrotCb(self, data):
        self.carrot_received = True
        self.carrot_pose = data.pose

    def stepCb(self, data):
        
        if self.carrot_received:
          
          print self.carrot_pose.position.x + data.pose.position.x
          
          new_msg = MultiDOFJointTrajectoryPoint()
          new_msg.transforms.append(Transform())
          new_msg.velocities.append(Twist())
          new_msg.accelerations.append(Twist())
          new_msg.transforms[0].translation.x = self.carrot_pose.position.x + data.pose.position.x
          new_msg.transforms[0].translation.y = self.carrot_pose.position.y + data.pose.position.y
          new_msg.transforms[0].translation.z = self.carrot_pose.position.z + data.pose.position.z

          new_msg.transforms[0].rotation.x = self.carrot_pose.orientation.x + data.pose.orientation.x
          new_msg.transforms[0].rotation.y = self.carrot_pose.orientation.y + data.pose.orientation.y
          new_msg.transforms[0].rotation.z = self.carrot_pose.orientation.z + data.pose.orientation.z
          new_msg.transforms[0].rotation.w = self.carrot_pose.orientation.w + data.pose.orientation.w

          print new_msg
          self.publisher.publish(new_msg)


if __name__ == "__main__":
    rospy.init_node('step_publisher')
    step_publisher = StepPublisher()
    rospy.spin()

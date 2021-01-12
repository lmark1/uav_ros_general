#!/usr/bin/env python
import copy
import math
import sys, os
import time

from geometry_msgs.msg import Pose, Point, Transform, Twist
from nav_msgs.msg import Path
from std_msgs.msg import String
from visualization_msgs.msg import Marker
import rospy
from topp_ros.srv import GetHelixPoints, GetHelixPointsResponse, \
    GetHelixPointsRequest, GenerateTrajectory, GenerateTrajectoryRequest, \
    GenerateTrajectoryResponse
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint, \
    MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint


class HelicalTrajectory():
    """ Generate and execute a helical trajectory.

        To avoid discontinuities, also generate a linear trajectory from
        the current carrot reference to the start of the helix.

        Requires the following scripts from the topp_ros package:

          get_helix_points.py
          generate_toppra_trajectory.py
    """
    def __init__(self):

        # Helix parameters
        self.r = 5
        self.angleStep = 1
        self.x0 = 0.0
        self.y0 = 0.0
        self.z0 = 2.0
        self.zf = 5.0
        self.deltaZ = 0.2

        self.velocities = 1.00
        self.accelerations = 0.5

        self.trajectory_sampling_period = 0.01

        # Define services
        self.trajectory_type = rospy.get_param("~trajectory_type",
            "generate_toppra_trajectory")
        self.request_trajectory_service = rospy.ServiceProxy(
            self.trajectory_type, GenerateTrajectory)
        self.get_helix_points_service = rospy.ServiceProxy(
            "get_helix_points", GetHelixPoints)
        
        # Define publishers
        self.trajectory_pub = rospy.Publisher('helix/trajectory',
            MultiDOFJointTrajectory, queue_size=1)
        self.trajectory_point_pub = rospy.Publisher('position_hold/trajectory',
            MultiDOFJointTrajectoryPoint, queue_size=1)

        # Subscribe to carrot
        self.carrot_sub = rospy.Subscriber('carrot/trajectory', 
            MultiDOFJointTrajectoryPoint, self.CarrotCb)
        self.carrot_status_sub = rospy.Subscriber('carrot/status', String, self.StatusCb)

        # Initialize flags
        self.carrot_received = False
        self.carrot_status = String()
        self.executing_trajectory = False
        self.trajectory_generated = False


    def JointTrajectory2MultiDofTrajectory(self, joint_trajectory):
        multi_dof_trajectory = MultiDOFJointTrajectory()

        for i in range(0, len(joint_trajectory.points)):
            temp_point = MultiDOFJointTrajectoryPoint()
            temp_transform = Transform()
            temp_transform.translation.x = joint_trajectory.points[i].positions[0]
            temp_transform.translation.y = joint_trajectory.points[i].positions[1]
            temp_transform.translation.z = joint_trajectory.points[i].positions[2]
            temp_transform.rotation.w = 1.0

            temp_vel = Twist()
            temp_vel.linear.x = joint_trajectory.points[i].velocities[0]
            temp_vel.linear.y = joint_trajectory.points[i].velocities[1]
            temp_vel.linear.z = joint_trajectory.points[i].velocities[2]

            temp_acc = Twist()
            temp_acc.linear.x = joint_trajectory.points[i].accelerations[0]
            temp_acc.linear.y = joint_trajectory.points[i].accelerations[1]
            temp_acc.linear.z = joint_trajectory.points[i].accelerations[2]

            temp_point.transforms.append(temp_transform)
            temp_point.velocities.append(temp_vel)
            temp_point.accelerations.append(temp_acc)

            multi_dof_trajectory.points.append(temp_point)

        return multi_dof_trajectory


    def StatusCb(self, data):
        if data.data != self.carrot_status:
          rospy.loginfo("Carrot status changed from %s to %s", self.carrot_status, data.data)
        self.carrot_status = data.data

    def CarrotCb(self, data):
        if self.carrot_received == False and self.carrot_status == "HOLD":
          self.carrot_data = data
          self.carrot_received = True

          rospy.logwarn("Carrot received in position hold mode. Generating trajectory")

          # Set up helical trajectory
          helix_request = GetHelixPointsRequest()
          helix_request.r = self.r
          helix_request.angleStep = self.angleStep
          helix_request.x0 = self.x0
          helix_request.y0 = self.y0
          helix_request.z0 = self.z0
          helix_request.zf = self.zf
          helix_request.deltaZ = self.deltaZ

          # Get the points
          helix_response = self.get_helix_points_service(helix_request)

          # Prepend the current carrot position
          first_point = JointTrajectoryPoint()
          initial_pose = data.transforms[0].translation
          first_point.positions = [initial_pose.x, initial_pose.y, initial_pose.z, 0]
          helix_response.helix_points.points.insert(0, first_point)

          # GetHelixPoints just returns the points. The dynamical part must be
          # provided by the user.
          
          dof = len(helix_response.helix_points.points[0].positions)
          helix_response.helix_points.points[0].velocities = [self.velocities]*dof
          helix_response.helix_points.points[0].accelerations = [self.accelerations]*dof

          # Now call the trajectory generation service
          trajectory_request = GenerateTrajectoryRequest()
          trajectory_request.waypoints = helix_response.helix_points
          trajectory_request.sampling_frequency = 100
          trajectory_response = self.request_trajectory_service(trajectory_request)

          # Repack the received trajectory to MultiDof
          self.multi_dof_trajectory = self.JointTrajectory2MultiDofTrajectory(
              trajectory_response.trajectory)

          self.trajectory_generated = True
          rospy.loginfo("Trajectory generated")

          self.trajectory_pub.publish(self.multi_dof_trajectory)
    

    def Run(self):

        while not rospy.is_shutdown():

            if self.trajectory_generated:
              if not self.executing_trajectory:
                self.executing_trajectory = True
                rospy.loginfo("Trajectory execution started")
              self.trajectory_point_pub.publish(self.multi_dof_trajectory.points.pop(0))
            rospy.sleep(self.trajectory_sampling_period)


if __name__ == "__main__":
    rospy.init_node("quad_helix_trajectory_node")
    a = HelicalTrajectory()
    a.Run()

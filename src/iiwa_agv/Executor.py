#!/usr/bin/env python

from __future__ import print_function
import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the joint trajectory control action, including the
# goal message and the result message.
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

class Executor:

    def __init__(self, file_path='../data/mbx_planned_trajectory.txt', action_ns='/iiwa/iiwa_controller/follow_joint_trajectory'):
        
        # file path
        self._file_path = file_path

        # action client
        self._action_client = actionlib.SimpleActionClient(action_ns, FollowJointTrajectoryAction)

        # wait for the action server until finally detecting
        rospy.loginfo("Start waiting for the action server")
        self._action_client.wait_for_server()
        rospy.loginfo("Server detected!")

    def read_trajectory(self):
        pass
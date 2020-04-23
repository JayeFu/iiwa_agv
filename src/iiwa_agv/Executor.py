#!/usr/bin/env python

import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the joint trajectory control action, including the
# goal message and the result message.
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

class Executor:

    def __init__(self, file_path='../data/fwx_planned_trajectory.txt', action_ns='/iiwa/iiwa_controller/follow_joint_trajectory'):
        
        # file path
        self._file_path = file_path

        # number of joints
        self._joint_num = 7

        # a dictionary containing all the values of the joints in time series
        self._joints_pos_dict = dict()

        for idx in range(self._joint_num):
            self._joints_pos_dict['joint_a'+str(idx+1)]=list()

        # a list containing the time series
        self._time_list = list()

        # action client
        self._action_client = actionlib.SimpleActionClient(action_ns, FollowJointTrajectoryAction)

        # wait for the action server until finally detecting
        rospy.loginfo("Start waiting for the action server")
        self._action_client.wait_for_server()
        rospy.loginfo("Server detected")

    def read_trajectory(self):
        with open(self._file_path,'r') as f:
            for line in f.readlines()[1:]:
                # convert string to float
                time_slice = map(float,line.split())
                # index 0 is time
                self._time_list.append(time_slice[0])
                for idx in range(self._joint_num):
                    # joint_a1 - time_slice[4]
                    # joint_a2 - time_slice[5]
                    # ...
                    # joint_a7 - time_slice[10]
                    self._joints_pos_dict['joint_a'+str(idx+1)].append(time_slice[idx+4])

    def init_pose(self):

        rospy.loginfo("Start initiating the pose")

        # a goal to be sent to action server
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(1.0)

        # a joint point in the trajectory
        trajPt = JointTrajectoryPoint()
        for idx in range(self._joint_num): # for each joint 
            joint_name = "joint_a"+str(idx+1)
            goal.trajectory.joint_names.append(joint_name)
            trajPt.positions.append(self._joints_pos_dict[joint_name][0])
            trajPt.velocities.append(0.0)
        trajPt.time_from_start = rospy.Duration(secs=2.0)

        # add the joint trajectory point to the goal
        goal.trajectory.points.append(trajPt)

        # send the goal to the action server
        self._action_client.send_goal(goal)

        # wait for the result
        rospy.loginfo("Start waiting for finishing initial pose")
        self._action_client.wait_for_result()
        rospy.loginfo("Waiting ends")

        # show the error code
        rospy.loginfo(self._action_client.get_result())

    def send_trajectory(self):

        rospy.loginfo("Start going to other trajectory points")

        # a goal to be sent to action server
        # this goal will contain all the joint trajectory points read from the txt file
        goal = FollowJointTrajectoryGoal()

        # add joint name
        for idx in range(self._joint_num): 
            goal.trajectory.joint_names.append("joint_a"+str(idx+1))

        # since the first pt has been realized, iteration will start from the second pt 
        for traj_idx in range(len(self._time_list)-1):
            # a joint point in the trajectory
            trajPt = JointTrajectoryPoint()
            for idx in range(self._joint_num):
                joint_name = "joint_a"+str(idx+1)
                trajPt.positions.append(self._joints_pos_dict[joint_name][traj_idx+1])
                trajPt.velocities.append(0.0)
            # time to reach the joint trajectory point specified
            trajPt.time_from_start = rospy.Duration(secs=2*self._time_list[traj_idx+1])
            # add the joint trajectory point to the goal
            goal.trajectory.points.append(trajPt)

        rospy.loginfo("goal has {} points to reach".format(len(goal.trajectory.points)))

        rospy.loginfo("each of them is ")

        for traj_idx in range(len(goal.trajectory.points)):
            print goal.trajectory.points[traj_idx]

        goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(2.0)

        # send the goal to the action server
        self._action_client.send_goal(goal)

        # wait for the result
        rospy.loginfo("Start waiting for go to other poses")
        self._action_client.wait_for_result()
        rospy.loginfo("Waiting ends")

        # show the error code
        rospy.loginfo(self._action_client.get_result())

    def go_to_last_pose(self):
        
        rospy.loginfo("Start going to the last pose")

        # a goal to be sent to action server
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(1.0)

        series_length = len(self._time_list)

        # a joint point in the trajectory
        trajPt = JointTrajectoryPoint()
        for idx in range(self._joint_num): # for each joint 
            joint_name = "joint_a"+str(idx+1)
            goal.trajectory.joint_names.append(joint_name)
            trajPt.positions.append(self._joints_pos_dict[joint_name][series_length-1])
            trajPt.velocities.append(0.0)
        trajPt.time_from_start = rospy.Duration(secs=3.0)

        # add the joint trajectory point to the goal
        goal.trajectory.points.append(trajPt)

        # send the goal to the action server
        self._action_client.send_goal(goal)

        # wait for the result
        rospy.loginfo("Start waiting for going to the last pose")
        self._action_client.wait_for_result()
        rospy.loginfo("Waiting ends")

        # show the error code
        rospy.loginfo(self._action_client.get_result())

    def go_to_target_pose(self, target_joint_values):

        rospy.loginfo("Start going to the target pose at {}".format(target_joint_values))

        # a goal to be sent to action server
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(1.0)

        # a joint point in the trajectory
        trajPt = JointTrajectoryPoint()
        for idx in range(self._joint_num): # for each joint 
            joint_name = "joint_a"+str(idx+1)
            goal.trajectory.joint_names.append(joint_name)
            trajPt.velocities.append(0.0)
        trajPt.positions = target_joint_values
        trajPt.time_from_start = rospy.Duration(secs=3.0)

        # add the joint trajectory point to the goal
        goal.trajectory.points.append(trajPt)

        # send the goal to the action server
        self._action_client.send_goal(goal)

        # wait for the result
        rospy.loginfo("Start waiting for going to the target pose")
        self._action_client.wait_for_result()
        rospy.loginfo("Waiting ends")

        # show the error code
        rospy.loginfo(self._action_client.get_result())

    def send_trajectory_one_by_one(self):
        rospy.loginfo("Start going to other trajectory points one by one")

        
        # since the first pt has been realized, iteration will start from the second pt 
        for traj_idx in range(len(self._time_list)-1):
            # a goal to be sent to action server
            # this goal will contain only one trajectory point
            goal = FollowJointTrajectoryGoal()

            # add joint name
            for idx in range(self._joint_num): 
                goal.trajectory.joint_names.append("joint_a"+str(idx+1))

            # a joint point in the trajectory
            trajPt = JointTrajectoryPoint()
            for idx in range(self._joint_num):
                joint_name = "joint_a"+str(idx+1)
                trajPt.positions.append(self._joints_pos_dict[joint_name][traj_idx+1])
                trajPt.velocities.append(0.0)
            # time to reach the joint trajectory point specified to 1.0 since this will be controlled by my enter
            trajPt.time_from_start = rospy.Duration(secs=1.0)
            # add the joint trajectory point to the goal
            goal.trajectory.points.append(trajPt)

            rospy.loginfo("At iteration {} goal has {} points to reach".format(traj_idx ,len(goal.trajectory.points)))

            rospy.loginfo("each of them is ")

            for traj_idx in range(len(goal.trajectory.points)):
                print goal.trajectory.points[traj_idx]

            goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(0.5)

            # send the goal to the action server
            self._action_client.send_goal(goal)

            # wait for the result
            rospy.loginfo("Start waiting for go to other poses")
            self._action_client.wait_for_result()
            rospy.loginfo("Waiting ends")

            # show the error code
            rospy.loginfo(self._action_client.get_result())

            raw_input()
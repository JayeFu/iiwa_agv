#!/usr/bin/env python  

import rospy

from iiwa_agv.Executor import Executor

def main():

    # start a new node
    rospy.init_node("executor")

    exe = Executor()

    # read the joint trajectory from txt file
    exe.read_trajectory()

    # initiate the pose with the first joint trajectory point
    exe.init_pose()

    # go to target pose
    # exe.go_to_target_pose([0.315593518306, -0.0583244396675, 0.316494316795, -0.0258618760329, 0.313625004658, 0.104796584025, 0.310698981177])

    # go to the last pose
    # exe.go_to_last_pose()

    # go to other joint trajectory points one by one
    # exe.send_trajectory_one_by_one()

    # go to other joint trajectory points
    exe.send_trajectory()



if __name__ == "__main__":
    main()
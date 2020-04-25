#!/usr/bin/env python  

import rospy
import sys
from Executor import Executor
from threading import Thread

def trajectory_func(ob):
    #rospy.loginfo(thread_name+'is running')
    # go to target pose
    # ob.go_to_target_pose([0.315593518306, -0.0583244396675, 0.316494316795, -0.0258618760329, 0.313625004658, 0.104796584025, 0.310698981177])

    # go to the last pose
    # ob.go_to_last_pose()

    # go to other joint trajectory points one by one
    #ob.send_trajectory_one_by_one(0.01)

    # go to other joint trajectory points
    ob.send_trajectory()
    #print(ob)

def vel_func(ob):
    #rospy.loginfo(thread_name+'is running')
    #send base velocity command
    ob.send_base_vel()
    #print(ob)

def main():

    # start a new node
    rospy.init_node("executor")

    exe = Executor()

    # read the joint trajectory from txt file
    exe.read_trajectory()

    # initiate the pose with the first joint trajectory point
    exe.init_pose()
    rospy.loginfo('please press enter to send to trajectory goal')
    raw_input()
    try:
        base_vel_thread=Thread(target=vel_func,args=(exe,))
        trajectory_thread=Thread(target=trajectory_func,args=(exe,))
        base_vel_thread.start()
        trajectory_thread.start()
        base_vel_thread.join()
        trajectory_thread.join()
        
    except:
        rospy.logerr('unable to start new thread')
    



if __name__ == "__main__":
    main()
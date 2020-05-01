#!/usr/bin/env python  

import rospy
import sys
from Executor import Executor
#from threading import Thread
#from gazebo_msgs.srv import *
#from gazebo_msgs.msg import ModelState
import math
#from geometry_msgs.msg import Pose
'''
def euler_to_quaternion(R,P,Y):
    p=Pose()
    p.orientation.w=math.cos(R/2)*math.cos(P/2)*math.cos(Y/2)+math.sin(R/2)*math.sin(P/2)*math.sin(Y/2)
    p.orientation.x=math.sin(R/2)*math.cos(P/2)*math.cos(Y/2)-math.cos(R/2)*math.sin(P/2)*math.sin(Y/2)
    p.orientation.y=math.cos(R/2)*math.sin(P/2)*math.cos(Y/2)+math.sin(R/2)*math.cos(P/2)*math.sin(Y/2)
    p.orientation.z=math.cos(R/2)*math.cos(P/2)*math.sin(Y/2)-math.sin(R/2)*math.sin(P/2)*math.cos(Y/2)
    return p.orientation

def trajectory_func(ob):
    rospy.loginfo('sending joint action goal')
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
    rospy.loginfo('publishing velocity command')
    #send base velocity command
    ob.send_base_vel()
    #print(ob)
'''
def main():

    # start a new node
    rospy.init_node("executor")

    rate = rospy.Rate(1.0)

    exe = Executor(file_path='/home/fjw/dual_ws/src/iiwa_agv/data/fwx_planned_trajectory_hou.txt')

    # listen to initial offset until success
    while not exe.listen_to_initial_offset():
        rospy.logerr("Waiting for listening to initial offset")
        rate.sleep()
    
    rospy.loginfo("Have successfully listened to initial offset")

    # read the joint trajectory from txt file
    exe.read_trajectory()

    # initiate the pose with the first joint trajectory point
    '''
    get_state_service = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    model = GetModelStateRequest()
    model.model_name = 'mm'
    robot_state = get_state_service(model)
    
    set_state_service = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    model = SetModelStateRequest()
    desired_state=ModelState()
    desired_state.model_name='fwx'
    desired_state.pose.position.x = exe._base_pose_list[0].x
    desired_state.pose.position.y = exe._base_pose_list[0].y
    desired_state.pose.orientation=euler_to_quaternion(0,0,math.pi/2-exe._base_pose_list[0].theta)
    set_state_service(desired_state)
    '''
    exe.init_pose()
    

    rospy.loginfo('please press enter to send to trajectory goal')
    raw_input()
    rospy.loginfo('sending joint action goal')
    # go to target pose
    # exe.go_to_target_pose([0.315593518306, -0.0583244396675, 0.316494316795, -0.0258618760329, 0.313625004658, 0.104796584025, 0.310698981177])

    # go to the last pose
    # exe.go_to_last_pose()

    # go to other joint trajectory points one by one
    #exe.send_trajectory_one_by_one(0.01)

    # go to other joint trajectory points
    exe.send_trajectory()
    #print(ob)
    '''
    try:
        base_vel_thread=Thread(target=vel_func,args=(exe,))
        trajectory_thread=Thread(target=trajectory_func,args=(exe,))
        base_vel_thread.start()
        trajectory_thread.start()
        base_vel_thread.join()
        trajectory_thread.join()
        
    except:
        rospy.logerr('unable to start new thread')
    '''



if __name__ == "__main__":
    main()
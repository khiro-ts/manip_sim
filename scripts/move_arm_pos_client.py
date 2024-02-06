#!/usr/bin/env python
import sys

import rospy
import actionlib
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from geometry_msgs.msg import Pose

from manip_sim.msg import MoveArmPosAction, MoveArmPosGoal, MoveArmPosResult, MoveArmPosFeedback

search_joint_state = [1.9151516, 1.7367947, -0.8863478, -2.8018741, 1.3113430, 2.9991712]

offset_wrt_aruco = [.3, 0., -.3, 0., 0., 0., 0.]

def move_arm_pos_client(target_type:int, goal_value):

    client = actionlib.SimpleActionClient('move_pos', MoveArmPosAction)
    print("waiting for move_pos action server...")
    client.wait_for_server()

    goal = MoveArmPosGoal()
    goal_joint_values = goal_value
    goal.target_type = target_type
    goal.target_values = goal_joint_values
    print(goal)

    client.send_goal(goal)
    client.wait_for_result()
    
    return client.get_result()

    
    
if __name__ == '__main__':
    
    try:
        rospy.init_node('move_arm_pos_client')
        result = move_arm_pos_client(0, search_joint_state)
        print("Result: ", result.success)
        #print("Feedback: ", result.current_joint_values)
    except rospy.ROSInterruptException:
        pass

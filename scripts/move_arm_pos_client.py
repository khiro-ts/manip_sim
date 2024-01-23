#!/usr/bin/env python
import sys

import rospy
import actionlib
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from manip_sim.msg import MoveArmPosAction, MoveArmPosGoal, MoveArmPosResult, MoveArmPosFeedback


def move_arm_pos_client():

    client = actionlib.SimpleActionClient('move_pos', MoveArmPosAction)
    print("waiting for move_pos action server...")
    client.wait_for_server()

    goal = MoveArmPosGoal()
    goal_joint_values = [1.0, 0.5, -0.2, 1.2, 0.0, 0.5]
    goal.target_joint_values = goal_joint_values
    print(goal)

    client.send_goal(goal)
    client.wait_for_result()
    
    return client.get_result()
    
if __name__ == '__main__':
    
    try:
        rospy.init_node('move_arm_pos_client')
        result = move_arm_pos_client()
        print("Result: ", result.success)
        #print("Feedback: ", result.current_joint_values)
    except rospy.ROSInterruptException:
        pass

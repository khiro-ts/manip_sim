#!/usr/bin/env python
import sys

import rospy
import moveit_commander
from moveit_commander import PlanningSceneInterface
from moveit_msgs.msg import RobotState, DisplayRobotState, DisplayTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import String

def move_to_joint_target(joint_names, joint_values):
    moveit_commander.roscpp_initialize(sys.argv)
    
    robot = moveit_commander.RobotCommander()
    group_name = "nova5_arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    scene = PlanningSceneInterface()
    
    move_group.set_joint_value_target(joint_values)
    
    ret, trajectory, planning_time, err = move_group.plan()
    rospy.loginfo(trajectory)
    move_group.execute(trajectory, wait=True)
    

if __name__ == '__main__':
    joint_states_topic = "/joint_states"
    joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
    goal_joint_values = [1.0, 0.5, -0.2, 1.2, 0.0, 0.5]
    
    rospy.init_node('moveit_sim', anonymous=True)

    try:
        joint_states = rospy.wait_for_message(joint_states_topic, JointState, timeout=5.0)
        current_joint_values = [joint_states.position[joint_states.name.index(joint)] for joint in joint_names]
        rospy.loginfo(current_joint_values)
        move_to_joint_target(joint_names, goal_joint_values)
    except rospy.ROSInterruptException:
        pass
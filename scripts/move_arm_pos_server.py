#!/usr/bin/env python
import sys

import rospy
import actionlib
import moveit_commander
from moveit_commander import PlanningSceneInterface
from moveit_msgs.msg import RobotState, DisplayRobotState, DisplayTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from manip_sim.msg import MoveArmPosAction, MoveArmPosGoal, MoveArmPosResult, MoveArmPosFeedback


class MoveArmPosServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer('move_pos', MoveArmPosAction, self.execute, False)
        self.joint_states_topic = "/joint_states"
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        self.server.start()

    def execute(self, goal):

        moveit_commander.roscpp_initialize(sys.argv)
    
        robot = moveit_commander.RobotCommander()
        group_name = "nova5_arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        scene = PlanningSceneInterface()

        move_group.set_joint_value_target(goal.target_joint_values)

        ret, trajectory, planning_time, err = move_group.plan()
        rospy.loginfo(trajectory)
        move_group.execute(trajectory, wait=True)
        
        current_joints = rospy.wait_for_message(self.joint_states_topic, JointState, timeout=5.0)
    
        feedback = MoveArmPosFeedback()
        feedback.current_joint_values = current_joints.position
        
        result = MoveArmPosResult()
        result.success = True

        self.server.set_succeeded(result)
    

if __name__ == '__main__':
    rospy.init_node('move_arm_pos_server')
    server = MoveArmPosServer()
    rospy.spin()

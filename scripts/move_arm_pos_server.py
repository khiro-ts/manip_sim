#!/usr/bin/env python
import sys

import rospy
import actionlib
import moveit_commander
from moveit_commander import PlanningSceneInterface
from moveit_msgs.msg import RobotState, DisplayRobotState, DisplayTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from geometry_msgs.msg import Pose

from manip_sim.msg import MoveArmPosAction, MoveArmPosGoal, MoveArmPosResult, MoveArmPosFeedback


class MoveArmPosServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer('move_pos', MoveArmPosAction, self.execute, False)
        self.joint_states_topic = "/joint_states"
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        self.server.start()
    
    def set_goal_pose(self, current_pose, offset):
        goal_pose = Pose()
        goal_pose.position.x = current_pose.position.x + offset[0]
        goal_pose.position.y = current_pose.position.y + offset[1]
        goal_pose.position.z = current_pose.position.z + offset[2]
        goal_pose.orientation.x = current_pose.orientation.x + offset[3]
        goal_pose.orientation.y = current_pose.orientation.y + offset[4]
        goal_pose.orientation.z = current_pose.orientation.z + offset[5]
        goal_pose.orientation.w = current_pose.orientation.w + offset[6]

        return goal_pose

    def execute(self, goal):

        moveit_commander.roscpp_initialize(sys.argv)
    
        robot = moveit_commander.RobotCommander()
        group_name = "nova5_arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        scene = PlanningSceneInterface()

        if goal.target_type == 0:
            move_group.set_joint_value_target(goal.target_values)
        elif goal.target_type == 1:
            move_group.set_pose_reference_frame("Link1")
            current_pose = move_group.get_current_pose(end_effector_link="Link6")
            rospy.loginfo("current pose: {}".format(current_pose))
            goal_pose = self.set_goal_pose(current_pose.pose, goal.target_values)
            rospy.loginfo("goal pose: {}".format(goal_pose))
            move_group.set_pose_target(goal_pose)
        else:
            rospy.logerr("goal target type undefined")
            result = MoveArmPosResult()
            result.success = False
            self.server.set_aborted(result)
            return

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

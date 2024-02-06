#!/usr/bin/env python
import sys

import rospy
import actionlib
import moveit_commander
from moveit_commander import PlanningSceneInterface
from moveit_msgs.msg import RobotState, DisplayRobotState, DisplayTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseStamped, Transform

from manip_sim.msg import MoveArmPosAction, MoveArmPosGoal, MoveArmPosResult, MoveArmPosFeedback


class MoveArmPosServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer('move_pos', MoveArmPosAction, self.execute, False)
        self.joint_states_topic = "/joint_states"
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        self.server.start()
    
    def set_goal_pose(self, target_pose, offset=[0.,0.,0.,0.,0.,0.,0.]):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "arm_base_link"
        if isinstance(target_pose, Pose):
            goal_pose.pose.position.x = target_pose.position.x + offset[0]
            goal_pose.pose.position.y = target_pose.position.y + offset[1]
            goal_pose.pose.position.z = target_pose.position.z + offset[2]
            goal_pose.pose.orientation.x = target_pose.orientation.x + offset[3]
            goal_pose.pose.orientation.y = target_pose.orientation.y + offset[4]
            goal_pose.pose.orientation.z = target_pose.orientation.z + offset[5]
            goal_pose.pose.orientation.w = target_pose.orientation.w + offset[6]
        elif isinstance(target_pose, tuple):
            goal_pose.pose.position.x = target_pose[0]
            goal_pose.pose.position.y = target_pose[1]
            goal_pose.pose.position.z = target_pose[2]
            goal_pose.pose.orientation.x = target_pose[3]
            goal_pose.pose.orientation.y = target_pose[4]
            goal_pose.pose.orientation.z = target_pose[5]
            goal_pose.pose.orientation.w = target_pose[6]
        
        return goal_pose


    def execute(self, goal):

        moveit_commander.roscpp_initialize(sys.argv)
    
        robot = moveit_commander.RobotCommander()
        group_name = "nova5_arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        scene = PlanningSceneInterface()

        if goal.target_type == 0:  # joint move
            move_group.set_joint_value_target(goal.target_values)
        elif goal.target_type == 1:  # pose move
            move_group.set_pose_reference_frame("arm_base_link")
            move_group.set_start_state_to_current_state()
            goal_pose = self.set_goal_pose(goal.target_values)
            rospy.loginfo("move pose: {}".format(goal_pose))
            move_group.set_pose_target(goal_pose)
        elif goal.target_type == 2:  # relative move
            move_group.set_pose_reference_frame("arm_base_link")
            current_pose = move_group.get_current_pose(end_effector_link="Link6")
            move_group.set_start_state_to_current_state()
            rospy.loginfo("current pose: {}".format(current_pose))
            goal_pose = self.set_goal_pose(current_pose.pose, goal.target_values)
            rospy.loginfo("rel move pose: {}".format(goal_pose))
            move_group.set_pose_target(goal_pose)
        else:
            rospy.logerr("goal target type undefined")
            result = MoveArmPosResult()
            result.success = False
            self.server.set_aborted(result)
            return

        ret, trajectory, planning_time, err = move_group.plan()
        if not ret:
            result = MoveArmPosResult()
            result.success = False
            self.server.set_aborted(result)
            return

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

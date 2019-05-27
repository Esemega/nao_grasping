#! /usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import time

from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *

def all_close(goal, actual, tolerance):
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

class NaoGrasping(object):
  """NaoGrasping"""
  def __init__(self):
    super(grasping, self).__init__()

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('nao_grasping', anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name1 = "right_arm"
    group_name2 = "left_arm"
    group_name3 = "right_leg"
    group_name4 = "left_leg"
    move_group1 = moveit_commander.MoveGroupCommander(group_name1)
    move_group1 = moveit_commander.MoveGroupCommander(group_name2)
    move_group1 = moveit_commander.MoveGroupCommander(group_name3)
    move_group1 = moveit_commander.MoveGroupCommander(group_name4)
display_trajectory_publisher = rospy.Publisher(
    '/move_group/display_planned_path',
    moveit_msgs.msg.DisplayTrajectory, queue_size=10)

## Getting Basic Information
planning_frame = move_group8.get_planning_frame()
print "Planning frame: %s" % planning_frame
eef_link = move_group2.get_end_effector_link()
eef_link_2 = move_group3.get_end_effector_link()
print "End effector link: %s" % eef_link
group_names = robot.get_group_names()
print "Available Planning Groups:", robot.get_group_names()
print "Printing robot state"
print robot.get_current_state()
print ""
print "Pulsa Enter para continuar..."
raw_input()

#########ADDING AN OBJECT#####

####Adding the grippable box
box_pose = geometry_msgs.msg.PoseStamped()
box_pose.header.frame_id = robot.get_planning_frame()
box_pose.header.stamp = rospy.Time.now()
#position
box_pose.pose.position.x = 0.12
box_pose.pose.position.y = -0.0710305712286
box_pose.pose.position.z = -0.00372045804698
#orientation:
box_pose.pose.orientation.w = 1.0


box_name="box"
box_size=(0.03, 0.03, 0.09)
scene.add_box(box_name, box_pose, box_size)

############################PRE-GRASP POSITION #############
pose_target = geometry_msgs.msg.Pose()
#position:
pose_target.position.x = 0.0657446163864
pose_target.position.y = -0.115435515057
pose_target.position.z = -0.00246913897985
#orientation:
pose_target.orientation.x = 0.70120366481
pose_target.orientation.y = 0.0560746719916
pose_target.orientation.z = 0.0540591379501
pose_target.orientation.w = 0.708693630014

# x: 0.0657446163864
#    y: -0.115435515057
#    z: -0.00246913897985
#  orientation:
#    x: 0.70120366481
#    y: 0.0560746719916
#    z: 0.0540591379501
#    w: 0.708693630014

group.set_pose_target(pose_target)

plan = group.plan()

rospy.sleep(5)

group.go(wait=True)

group.stop()


############################OBJECT POSITION #############
pose_target = geometry_msgs.msg.Pose()
#position:
pose_target.position.x = 0.0670417715521
pose_target.position.y = -0.0710305712286
pose_target.position.z = -0.00372045804698
#orientation:
pose_target.orientation.x = 0.701316476786
pose_target.orientation.y = 0.0547707806601
pose_target.orientation.z = 0.0567321994159
pose_target.orientation.w = 0.708474994988

# x: 0.0670417715521
#     y: -0.0710305712286
#     z: -0.00372045804698
#   orientation:
#     x: 0.701316476786
#     y: 0.0547707806601
#     z: 0.0567321994159
#     w: 0.708474994988



group.set_pose_target(pose_target)

plan = group.plan()

rospy.sleep(5)

group.go(wait=True)

group.stop()
group.clear_pose_targets()

moveit_commander.roscpp_shutdown()

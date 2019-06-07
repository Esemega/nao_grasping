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

class NaoTableGrasping(object):
  """NaoTableGrasping"""
  def __init__(self):
    super(NaoTableGrasping, self).__init__()

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('nao_grasping', anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name1 = "right_arm"
    group_name1_1="right_arm_hand"
    group_name2 = "left_arm"
    group_name2_1="left_arm_hand"
    group_name3 = "right_leg"
    group_name4 = "left_leg"
    group_name5 = "both_arms"
    group_name6 = "both_legs"
    move_group1 = moveit_commander.MoveGroupCommander(group_name1)
    move_group2 = moveit_commander.MoveGroupCommander(group_name2)
    move_group1_1 = moveit_commander.MoveGroupCommander(group_name1_1)
    move_group2_1 = moveit_commander.MoveGroupCommander(group_name2_1)
    move_group3 = moveit_commander.MoveGroupCommander(group_name3)
    move_group4 = moveit_commander.MoveGroupCommander(group_name4)
    move_group5 = moveit_commander.MoveGroupCommander(group_name5)
    move_group6 = moveit_commander.MoveGroupCommander(group_name6)

    display_trajectory_publisher = rospy.Publisher(
    '/move_group/display_planned_path',
    moveit_msgs.msg.DisplayTrajectory, queue_size=10)

    ## Getting Basic Information
    planning_frame = move_group1.get_planning_frame()
    print "Planning frame: %s" % planning_frame
    planning_frame_robot = robot.get_planning_frame()
    print "Planning frame: %s" % planning_frame
    eef_link_r = move_group1.get_end_effector_link()
    print "End effector link for right arm: %s" % eef_link_r
    eef_link_l = move_group2.get_end_effector_link()
    print "End effector link for left arm: %s" % eef_link_l
    group_names = robot.get_group_names()
    print "Available Planning Groups:", robot.get_group_names()
    print "Printing robot state"
    print robot.get_current_state()
    print ""

    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.move_group1 = move_group1
    self.move_group2 = move_group2
    self.move_group1_1 = move_group1_1
    self.move_group2_1 = move_group2_1
    self.move_group3 = move_group3
    self.move_group4 = move_group4
    self.move_group5 = move_group5
    self.move_group6 = move_group6
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.planning_frame_robot = planning_frame_robot
    self.eef_link_r = eef_link_r
    self.eef_link_l = eef_link_l
    self.group_names = group_names
    self.group_name1 = group_name1

  def adding_objects(self):
     scene = self.scene
     planning_frame_robot = self.planning_frame_robot

     rospy.sleep(5)
     ####Adding a table
     table_pose = geometry_msgs.msg.PoseStamped()
     table_pose.header.frame_id = planning_frame_robot
     table_pose.header.stamp = rospy.Time.now()
     table_pose.pose.orientation.w = 1.0;
     table_pose.pose.position.x = 0.22;
     table_pose.pose.position.y = 0.0;
     table_pose.pose.position.z = -0.02;

     table_name="table"
     # table_size=(0.2, 0.5, 0.01)
     table_size=(0.2, 0.4, 0.01)
     scene.add_box(table_name, table_pose, table_size)

     rospy.sleep(5)

     ####Adding the grippable box
     box_pose = geometry_msgs.msg.PoseStamped()
     box_pose.header.frame_id = planning_frame_robot
     box_pose.header.stamp = rospy.Time.now()
     box_pose.pose.orientation.w = 1.0;
     box_pose.pose.position.x = 0.1840;
     box_pose.pose.position.y = -0.1256;
     box_pose.pose.position.z = 0.0300;

     box_name="grippable box"
     box_size=(0.015, 0.015, 0.09)
     scene.add_box(box_name, box_pose, box_size)

     self.box_name = box_name
     self.box_pose = box_pose
     self.box_size = box_size


  def nao_grasping_func(self):
    move_group1 = self.move_group1
    move_group2 = self.move_group2
    move_group1_1 = self.move_group1_1
    move_group2_1 = self.move_group2_1
    move_group3 = self.move_group3
    move_group4 = self.move_group4
    move_group5 = self.move_group5
    move_group6 = self.move_group6
    scene = self.scene
    box_name = self.box_name
    robot = self.robot
    eef_link_r = self.eef_link_r
    eef_link_l = self.eef_link_l
    group_names = self.group_names
    group_name1 = self.group_name1
    box_pose = self.box_pose

    #obtaining the initial pose of right arm
    initial_position1 = geometry_msgs.msg.Pose()
    initial_position1=move_group1.get_current_pose()

    ############################ MOVING RIGHT ARM #############
    ###### pose control ########
    #RIGHT_ARM
    pose_target1 = geometry_msgs.msg.Pose()
    #position:
    pose_target1.position.x = 0.124194647206
    pose_target1.position.y = -0.138372787579
    pose_target1.position.z = 0.0190369262287
    #orientation:
    pose_target1.orientation.x = 0.704102253044
    pose_target1.orientation.y = -0.0293804604563
    pose_target1.orientation.z = 0.033557197715
    pose_target1.orientation.w = 0.708696493771

    move_group1.set_pose_target(pose_target1)

    plan = move_group1.plan()
    move_group1.go(wait=True)

    ############################ OPENNING RIGHT HAND #############
    ######articular control ########
    joint_goal = move_group1.get_current_joint_values()
    joint_goal[5] = 1
    move_group1.go(joint_goal, wait=True)

    ############################ MOVING RIGHT ARM #############
    ###### pose control ########
    #RIGHT_ARM
    pose_target2 = geometry_msgs.msg.Pose()
    #position:
    pose_target2.position.x = 0.171615221436
    pose_target2.position.y = -0.136012206627
    pose_target2.position.z = 0.0375827851372
    #orientation:
    pose_target2.orientation.x = 0.955362875004
    pose_target2.orientation.y = -0.00972021742972
    pose_target2.orientation.z = 0.0431144239835
    pose_target2.orientation.w = 0.292110323135

    move_group1.set_pose_target(pose_target2)

    plan1 = move_group1.plan()
    move_group1.go(wait=True)

    ############################ CLOSING RIGHT HAND #############
    ######articular control ########
    joint_goal1 = move_group1.get_current_joint_values()
    joint_goal1[5] = 0
    move_group1.go(joint_goal1, wait=True)

    rospy.sleep(1)

    #################### ATACHING THE BOX TO RIGHT HAND ###########
    grasping_group = "right_hand"
    touch_links = robot.get_link_names(group=grasping_group)
    scene.attach_box(eef_link_r, box_name, touch_links=touch_links)

    rospy.sleep(2)

    ######################### MOVING RIGHT ARM #############
    ###### pose control ########
    #RIGHT_ARM
    pose_target5 = geometry_msgs.msg.Pose()
    #position:
    pose_target5.position.x = 0.152747754169
    pose_target5.position.y = -0.0369405272717
    pose_target5.position.z = 0.039242791345
    #orientation:
    pose_target5.orientation.x = 0.670235609083
    pose_target5.orientation.y = 0.265155388852
    pose_target5.orientation.z = 0.269667077353
    pose_target5.orientation.w = 0.638558153555

    move_group1.set_pose_target(pose_target5)

    plan4 = move_group1.plan()
    move_group1.go(wait=True)

    ############################ OPENNING RIGHT HAND #############
    ######articular control ########
    joint_goal2 = move_group1.get_current_joint_values()
    joint_goal2[5] = 1
    move_group1.go(joint_goal2, wait=True)

    rospy.sleep(3)

    ################## DETACHING THE BOX FROM RIGHT HAND ###########
    scene.remove_attached_object(eef_link_r, name=box_name)
    rospy.sleep(3)

    ################# MOVING RIGHT ARM TO INITIAL POSE #############
    ###### pose control ########
    #RIGHT_ARM
    move_group1.set_pose_target(initial_position1)

    plan5 = move_group1.plan()
    move_group1.go(wait=True)

  def ending_program(self):
      move_group1 = self.move_group1
      scene = self.scene
      box_name = self.box_name
      robot = self.robot

      ##################### ENDING PROGRAM ##################
      rospy.sleep(3)
      scene.remove_world_object(box_name)
      scene.remove_world_object("table")
      rospy.sleep(1)

      move_group1.stop()
      move_group1.clear_pose_targets()
      moveit_commander.roscpp_shutdown()

def main():
  try:
    print ""
    print "-------------------------------------------------"
    print "Joint and pose control with NAO: objets manipulation"
    print "-------------------------------------------------"
    print ""

    tutorial = NaoTableGrasping()
    tutorial.adding_objects()
    tutorial.nao_grasping_func()
    tutorial.ending_program()


    print "Control successfully completed!"
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()

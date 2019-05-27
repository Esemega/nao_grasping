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
    super(NaoGrasping, self).__init__()

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('nao_grasping', anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name1 = "right_arm"
    group_name2 = "left_arm"
    group_name3 = "right_leg"
    group_name4 = "left_leg"
    group_name5 = "both_arms"
    group_name6 = "both_legs"
    move_group1 = moveit_commander.MoveGroupCommander(group_name1)
    move_group2 = moveit_commander.MoveGroupCommander(group_name2)
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
    self.move_group3 = move_group3
    self.move_group4 = move_group4
    self.move_group5 = move_group5
    self.move_group6 = move_group6
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link_r = eef_link_r
    self.eef_link_l = eef_link_l
    self.group_names = group_names
    self.group_name1 = group_name1

  def nao_grasping_func(self):
    move_group1 = self.move_group1
    move_group2 = self.move_group2
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

    ############################ MOVING LEGS #############
    ######articular control ########
    joint_goal = move_group6.get_current_joint_values()
    #joint_goal[0] = 0 #-0.55
    #joint_goal[1] = 0 #0.2
    joint_goal[2] = -0.6 #-1.4
    joint_goal[3] = 1.7 #2.1
    joint_goal[4] = -1.1 #-1.1
    #joint_goal[5] = 0 #0
    #joint_goal[6] = 0 #-0.55
    #joint_goal[7] = 0 #0.2
    joint_goal[8] = -0.6 #-1.4
    joint_goal[9] = 1.7 #2.1
    joint_goal[10] = -1.1 #-1.1
    #joint_goal[11] = 0 #0
    print("Joint control. Nao bends down...")
    move_group6.go(joint_goal, wait=True)

    ############################ MOVING LEFT ARM #############
    ###### pose control ########
    #LEFT_ARM
    pose_target1 = geometry_msgs.msg.Pose()
    #position:
    pose_target1.position.x = 0.202673056955
    pose_target1.position.y = 0.107829939521
    pose_target1.position.z = 0.0171779146918
    #orientation:
    pose_target1.orientation.x = -0.0281539498024
    pose_target1.orientation.y = 0.165052413403
    pose_target1.orientation.z = -0.0155491917426
    pose_target1.orientation.w = 0.985760254107

    print("Pose control. Nao move the left arm...")
    move_group2.set_pose_target(pose_target1)

    plan = move_group2.plan()
    move_group2.go(wait=True)

    ############################ OPENNING LEFT HAND #############
    ######articular control ########
    joint_goal1 = move_group2.get_current_joint_values()
    # joint_goal1[0] = -1.3
    # joint_goal1[1] = 0
    # joint_goal1[2] = 1.6
    # joint_goal1[3] = -0.04
    # joint_goal1[4] = 0
    joint_goal1[5] = 1
    print("Joint control. Nao opens the left hand...")
    move_group2.go(joint_goal1, wait=True)

    rospy.sleep(1)

    ####################ADDING A BOX ON LEFT HAND ###########
    print "Adding a box"
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "LFootBumperRight_frame"
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.x = 0.16
    box_pose.pose.position.z = 0.22
    box_pose.pose.position.y = 0.07
    box_name = "box"
    scene.add_box(box_name, box_pose, size=(0.02, 0.05, 0.08))

    rospy.sleep(1)

    ############################ CLOSING LEFT HAND #############
    ######articular control ########
    joint_goal2 = move_group2.get_current_joint_values()
    # joint_goal1[0] = -1.3
    # joint_goal1[1] = 0
    # joint_goal1[2] = 1.6
    # joint_goal1[3] = -0.04
    # joint_goal1[4] = 0
    joint_goal2[5] = 0
    print("Joint control. Nao closes the left hand...")
    move_group2.go(joint_goal2, wait=True)

    rospy.sleep(1)

    #################### ATACHING THE BOX TO LEFT HAND ###########
    grasping_group = "left_hand"
    touch_links = robot.get_link_names(group=grasping_group)
    scene.attach_box(eef_link_l, box_name, touch_links=touch_links)

    rospy.sleep(2)

    ############################ ORIENTING LEFT ARM #############
    ###### pose control ########
    #LEFT_ARM
    pose_target2 = geometry_msgs.msg.Pose()
    #position:
    pose_target2.position.x = 0.199460350307
    pose_target2.position.y = 0.0342562939196
    pose_target2.position.z = 0.0368955451157
    #orientation:
    pose_target2.orientation.x = -0.737522182926
    pose_target2.orientation.y = 0.208155423737
    pose_target2.orientation.z = 0.00409356380616
    pose_target2.orientation.w = 0.642429445151

    print("Pose control. Nao orient the left arm...")
    move_group2.set_pose_target(pose_target2)

    plan = move_group2.plan()
    move_group2.go(wait=True)

    ############################ MOVING RIGHT ARM #############
    ###### pose control ########
    #RIGHT_ARM
    pose_target3 = geometry_msgs.msg.Pose()
    #position:
    pose_target3.position.x = 0.183396317344
    pose_target3.position.y = -0.0916722423507
    pose_target3.position.z = 0.0415431876627
    #orientation:
    pose_target3.orientation.x = 0.972374159386
    pose_target3.orientation.y = -0.0239793232233
    pose_target3.orientation.z = 0.0777135563625
    pose_target3.orientation.w = 0.218801483938

    print("Pose control. Nao move the right arm and opens the right hand...")
    move_group1.set_pose_target(pose_target3)

    plan = move_group1.plan()
    move_group1.go(wait=True)

    ############################ MOVING RIGHT ARM #############
    ###### pose control ########
    #RIGHT_ARM
    pose_target4 = geometry_msgs.msg.Pose()
    #position:
    pose_target4.position.x = 0.189608196856
    pose_target4.position.y = -0.0606931187519
    pose_target4.position.z = 0.0288910344956
    #orientation:
    pose_target4.orientation.x = 0.982193318827
    pose_target4.orientation.y = 0.0675675183282
    pose_target4.orientation.z = -0.00215466763272
    pose_target4.orientation.w = 0.175289110689

    print("Pose control. Nao move the right arm...")
    move_group1.set_pose_target(pose_target4)

    plan = move_group1.plan()
    move_group1.go(wait=True)

    ############################ CLOSING RIGHT HAND #############
    ######articular control ########
    joint_goal5 = move_group1.get_current_joint_values()
    # joint_goal5[0] = -1.3
    # joint_goal5[1] = 0
    # joint_goal5[2] = 1.6
    # joint_goal5[3] = -0.04
    # joint_goal5[4] = 0
    joint_goal5[5] = 0
    print("Joint control. Nao closes the right hand...")
    move_group1.go(joint_goal5, wait=True)

    rospy.sleep(1)


##################### ENDING PROGRAM ##################
    rospy.sleep(3)
    scene.remove_attached_object(eef_link_l, name=box_name)
    rospy.sleep(1)
    scene.remove_world_object(box_name)
    rospy.sleep(1)

    move_group1.stop()
    move_group2.stop()
    move_group6.stop()
    move_group1.clear_pose_targets()
    move_group2.clear_pose_targets()
    move_group6.clear_pose_targets()
    moveit_commander.roscpp_shutdown()


def main():
  try:
    print ""
    print "-------------------------------------------------"
    print "Joint and pose control with NAO: objets manipulation"
    print "-------------------------------------------------"
    print ""

    tutorial = NaoGrasping()
    tutorial.nao_grasping_func()

    print "Control succedssfully completed!"
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()

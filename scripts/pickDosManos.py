#!/usr/bin/env python
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

class MoveGroupPythonIntefaceTutorial(object):
  """MoveGroupPythonIntefaceTutorial"""
  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name8 = "both_arms"
    group_name9 = "both_legs"
    group_name2 = "left_arm"
    move_group9 = moveit_commander.MoveGroupCommander(group_name9)
    move_group8 = moveit_commander.MoveGroupCommander(group_name8)
    move_group2 = moveit_commander.MoveGroupCommander(group_name2)

    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    ## Getting Basic Information
    planning_frame = move_group8.get_planning_frame()
    print "Planning frame: %s" % planning_frame
    eef_link = move_group2.get_end_effector_link()
    print "End effector link: %s" % eef_link
    group_names = robot.get_group_names()
    print "Available Planning Groups:", robot.get_group_names()
    print "Printing robot state"
    print robot.get_current_state()
    print ""

    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.move_group9 = move_group9
    self.move_group8 = move_group8
    self.move_group2 = move_group2
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names

  def go_to_joint_state(self):
    move_group9 = self.move_group9
    move_group8 = self.move_group8
    move_group2 = self.move_group2
    scene = self.scene

    #PIERNAS
    joint_goal = move_group9.get_current_joint_values()
    joint_goal[0] = 0 #-0.55
    joint_goal[1] = 0 #0.2
    joint_goal[2] = -0.6 #-1.4
    joint_goal[3] = 1.7 #2.1
    joint_goal[4] = -1.1 #-1.1
    joint_goal[5] = 0 #0

    joint_goal[6] = 0 #-0.55
    joint_goal[7] = 0 #0.2
    joint_goal[8] = -0.6 #-1.4
    joint_goal[9] = 1.7 #2.1
    joint_goal[10] = -1.1 #-1.1
    joint_goal[11] = 0 #0

    print("se agacha ...")
    raw_input()
    move_group9.go(joint_goal, wait=True)

    #BRAZOS
    joint_goal1 = move_group8.get_current_joint_values()
    joint_goal1[0] = -0.7
    joint_goal1[1] = 0
    joint_goal1[2] = 0
    joint_goal1[3] = -0.8
    joint_goal1[4] = 0
    joint_goal1[5] = 1

    joint_goal1[6] = -0.7
    joint_goal1[7] = 0
    joint_goal1[8] = 0
    joint_goal1[9] = 0.8
    joint_goal1[10] = 0
    joint_goal1[11] = 1

    print("mueve los dos brazos ...")
    raw_input()
    move_group8.go(joint_goal1, wait=True)

    #BRAZOS
    joint_goal1 = move_group8.get_current_joint_values()
    joint_goal1[0] = -0.7
    joint_goal1[1] = -0.31
    joint_goal1[2] = 0
    joint_goal1[3] = -0.8
    joint_goal1[4] = 0
    joint_goal1[5] = 1

    joint_goal1[6] = -0.7
    joint_goal1[7] = 0.31
    joint_goal1[8] = 0
    joint_goal1[9] = 0.8
    joint_goal1[10] = 0
    joint_goal1[11] = 1

    print("cogiendo caja ...")
    raw_input()
    move_group8.go(joint_goal1, wait=True)

    box_name = self.box_name
    scene = self.scene

    rospy.sleep(1)
    #definicion de la mesa soporte
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "LFootBumperRight_frame"
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.x = 0.16
    box_pose.pose.position.z = 0.25
    box_pose.pose.position.y = -0.035
    box_name = "box"
    scene.add_box(box_name, box_pose, size=(0.09, 0.09, 0.09))

    rospy.sleep(2)

    #attacher de la caja
    robot = self.robot
    #scene = self.scene
    eef_link = self.eef_link
    group_names = self.group_names
    grasping_group = "left_arm"
    touch_links = robot.get_link_names(group=grasping_group)
    scene.attach_box(eef_link, box_name, touch_links=touch_links)

    rospy.sleep(1)

    #PIERNAS
    joint_goal = move_group9.get_current_joint_values()
    #left leg
    joint_goal[0] = 0 #-0.55
    joint_goal[1] = 0 #0.2
    joint_goal[2] = 0 #-1.4
    joint_goal[3] = 0 #2.1
    joint_goal[4] = 0 #-1.1
    joint_goal[5] = 0 #0
    #right leg
    joint_goal[6] = 0 #-0.55
    joint_goal[7] = 0 #0.2
    joint_goal[8] = 0 #-1.4
    joint_goal[9] = 0 #2.1
    joint_goal[10] = 0 #-1.1
    joint_goal[11] = 0 #0

    print("se levanta con la caja ...")
    raw_input()
    move_group9.go(joint_goal, wait=True)

    rospy.sleep(3)
    scene.remove_attached_object(eef_link, name=box_name)
    rospy.sleep(2)
    scene.remove_world_object(box_name)
    rospy.sleep(1)

def main():
  try:
    print ""
    print "-------------------------------------------------"
    print "Control articular del NAO: manipulacion de objetos"
    print "-------------------------------------------------"

    tutorial = MoveGroupPythonIntefaceTutorial()
    #utorial.add_box()
    tutorial.go_to_joint_state()

    print "Control completado!"
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()

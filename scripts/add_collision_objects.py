#! /usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import shape_msgs

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('add_collision_object', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "right_arm"
group = moveit_commander.MoveGroupCommander(group_name)
display_trajectory_publisher = rospy.Publisher(
    '/move_group/display_planned_path',
    moveit_msgs.msg.DisplayTrajectory, queue_size=20)

planning_frame = group.get_planning_frame()
print "============ Reference frame: %s" % planning_frame

# We can also print the name of the end-effector link for this group:
eef_link = group.get_end_effector_link()
print "============ End effector: %s" % eef_link

# We can get a list of all the groups in the robot:
group_names = robot.get_group_names()
print "============ Robot Groups:", robot.get_group_names()

#You can also get the current Pose of the end-effector of the robot, like this:
print "============ Current Pose:", group.get_current_pose()

# Sometimes for debugging it is useful to print the entire state of the
# robot:
print "============ Printing robot state"
print robot.get_current_state()
print ""

####Adding a big box or table
table_pose = geometry_msgs.msg.PoseStamped()
table_pose.header.frame_id = robot.get_planning_frame()
table_pose.header.stamp = rospy.Time.now()
table_pose.pose.orientation.w = 1.0;
table_pose.pose.position.x = 0.22;
table_pose.pose.position.y = 0.0;
table_pose.pose.position.z = -0.21;

table_name="table"
table_size=(0.2, 0.5, 0.3)
scene.add_box(table_name, table_pose, table_size)

rospy.sleep(5)

####Adding the grippable box
box_pose = geometry_msgs.msg.PoseStamped()
box_pose.header.frame_id = robot.get_planning_frame()
box_pose.header.stamp = rospy.Time.now()
box_pose.pose.orientation.w = 1.0;
box_pose.pose.position.x = 0.15;
box_pose.pose.position.y = -0.09;
box_pose.pose.position.z = -0.01;

#box_only = (0.07 -0.09 0.05)

box_name="box"
box_size=(0.03, 0.03, 0.09)
scene.add_box(box_name, box_pose, box_size)

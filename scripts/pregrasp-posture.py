#! /usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('planning_script', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "right_arm"
group = moveit_commander.MoveGroupCommander(group_name)
display_trajectory_publisher = rospy.Publisher(
    '/move_group/display_planned_path',
    moveit_msgs.msg.DisplayTrajectory, queue_size=10)

# We can get the name of the reference frame for this robot:
planning_frame = group.get_planning_frame()
print "============ Reference frame: %s" % planning_frame

# We can also print the name of the end-effector link for this group:
eef_link = group.get_end_effector_link()
print "============ End effector: %s" % eef_link

# We can get a list of all the groups in the robot:
group_names = robot.get_group_names()
print "============ Robot Groups:", robot.get_group_names()

#You can get the current values of the joints, like this:
print "============ Current Joint Values:", group.get_current_joint_values()
#You can also get the current Pose of the end-effector of the robot, like this:
print "============ Current Pose:", group.get_current_pose()

# Sometimes for debugging it is useful to print the entire state of the
# robot:
print "============ Printing robot state"
print robot.get_current_state()
print ""

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

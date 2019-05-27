#! /usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs
import geometry_msgs
import shape_msgs

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('add_collision_object', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "right_arm"
group = moveit_commander.MoveGroupCommander(group_name)
#display_trajectory_publisher = rospy.Publisher(
#    '/move_group/display_planned_path',
#    moveit_msgs.msg.DisplayTrajectory, queue_size=10)

rospy.sleep(5)

#box_pose = geometry_msgs.msg.PoseStamped()
#box_pose.header.frame_id = "right_arm"
#box_pose.pose.orientation.w = 1.0

# box_pose = geometry_msgs.Pose()
# box_pose.orientation.w = 1.0;
# box_pose.position.x = 0.0;
# box_pose.position.y = -0.4;
# box_pose.position.z = -0.4;
# box_name = "box"
# scene.add_box(box_name, box_pose, size=(0.1, 0.1, 0.1))

cylinder = moveit_msgs.msg.CollisionObject()
cylinder.id = "cylinder";

primitive = shape_msgs.msg.SolidPrimitive()
primitive.type = shape_msgs.msg.SolidPrimitive.CYLINDER;
#primitive.dimensions.resize(3);
primitive.dimensions.append(0.6)# CYLINDER_HEIGHT
primitive.dimensions.append(0.2)# CYLINDER_RADIUS

pose = geometry_msgs.msg.Pose()
pose.orientation.w = 1.0;
pose.position.x = 0.0;
pose.position.y = -0.4;
pose.position.z = -0.4;

cylinder.primitives.append(primitive);
cylinder.primitive_poses.append(pose);
cylinder.operation = cylinder.ADD;

collision_objects = [moveit_msgs.msg.CollisionObject()]
collision_objects.append(cylinder);

### FALLA ESTA LINEA!!!!
scene.addCollisionObjects(collision_objects);

#================================================
#C++
# moveit::planning_interface::PlanningSceneInterface
# current_scene;
# sleep(5.0);
#
# moveit_msgs::CollisionObject cylinder;
# cylinder.id = "seven_dof_arm_cylinder";
#
# shape_msgs::SolidPrimitive primitive;
# primitive.type = primitive.CYLINDER;
# primitive.dimensions.resize(3);
# primitive.dimensions[0] = 0.6;
# primitive.dimensions[1] = 0.2;
#
# geometry_msgs::Pose pose;
# pose.orientation.w = 1.0;
# pose.position.x = 0.0;
# pose.position.y = -0.4;
# pose.position.z = -0.4;
#
# cylinder.primitives.push_back(primitive);
# cylinder.primitive_poses.push_back(pose);
# cylinder.operation = cylinder.ADD;

#std::vector<moveit_msgs::CollisionObject>
#collision_objects;
#collision_objects.push_back(cylinder);

#current_scene.addCollisionObjects(collision_objects);

#-------------------------------------

#Python

#box_pose = geometry_msgs.msg.PoseStamped()
#box_pose.header.frame_id = "panda_leftfinger"
#box_pose.pose.orientation.w = 1.0
#box_name = "box"
#scene.add_box(box_name, box_pose, size=(0.1, 0.1, 0.1))

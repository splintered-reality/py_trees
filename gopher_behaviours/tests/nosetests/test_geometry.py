#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/gopher_crazy_hospital/py_trees/license/LICENSE
#

##############################################################################
# Imports
##############################################################################

# enable some python3 compatibility options:
# (unicode_literals not compatible with python2 uuid module)
from __future__ import absolute_import, print_function

from nose.tools import assert_raises, assert_almost_equal
import geometry_msgs.msg as geometry_msgs
import gopher_behaviours.park as park
import gopher_behaviours.transform_utilities as transform_utilities
import math
import os
import rocon_console.console as console

##############################################################################
# Methods
##############################################################################

def print_summary(pose_park_rel_map, pose_park_start_rel_map, distance_to_park, point_to_park_angle, orient_with_park_angle):
    print_geometry_poses(pose_park_start_rel_map, pose_park_rel_map)
    print("")
    print("  Distance to Park    : %s" % distance_to_park)
    print("  Point to Park Angle : %s" % point_to_park_angle)
    print("  Orient w/ Park Angle: %s" % orient_with_park_angle)
    print("")

def print_geometry_poses(pose1, pose2):
    print(console.green + "[%s, %s, %s][x:%s y:%s z:%s w:%s] -> [%s, %s, %s][x:%s y:%s z:%s w:%s]" % (
        pose1.position.x,
        pose1.position.y,
        pose1.position.z,
        pose1.orientation.x,
        pose1.orientation.y,
        pose1.orientation.z,
        pose1.orientation.w,
        pose2.position.x,
        pose2.position.y,
        pose2.position.z,
        pose2.orientation.x,
        pose2.orientation.y,
        pose2.orientation.z,
        pose2.orientation.w
        ) + console.reset
    )

##############################################################################
# Tests
##############################################################################


#def test_parking_geometry():
#     print(console.bold + "\n****************************************************************************************" + console.reset)
#     print(console.bold + "* Parking Geometry " + console.reset)
#     print(console.bold + "****************************************************************************************\n" + console.reset)
#
#     # 1
#     pose_park_start_rel_map = geometry_msgs.Pose()
#     pose_park_start_rel_map.position = geometry_msgs.Point(0, 0, 0)
#     pose_park_start_rel_map.orientation = geometry_msgs.Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
#     pose_park_rel_map = geometry_msgs.Pose()
#     pose_park_rel_map.position = geometry_msgs.Point(0, 1, 0)
#     pose_park_rel_map.orientation = geometry_msgs.Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
#
#     (distance_to_park, point_to_park_angle, orient_with_park_angle) = park.compute_parking_geometry(pose_park_start_rel_map, pose_park_rel_map)
#     print_summary(pose_park_rel_map, pose_park_start_rel_map, distance_to_park, point_to_park_angle, orient_with_park_angle)
#
#     assert(distance_to_park == 1.0)
#     assert_almost_equal(point_to_park_angle, 1.5707963279, 3)
#     assert_almost_equal(orient_with_park_angle, -1.5707963279, 3)
#
#     # 2
#     pose_park_start_rel_map = geometry_msgs.Pose()
#     pose_park_start_rel_map.position = geometry_msgs.Point(0, 0, 0)
#     pose_park_start_rel_map.orientation = geometry_msgs.Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
#     pose_park_rel_map = geometry_msgs.Pose()
#     pose_park_rel_map.position = geometry_msgs.Point(1, 1, 0)
#     pose_park_rel_map.orientation = geometry_msgs.Quaternion(x=0.0, y=0.0, z=1.0, w=0.0)  # 180 degrees
#
#     (distance_to_park, point_to_park_angle, orient_with_park_angle) = park.compute_parking_geometry(pose_park_start_rel_map, pose_park_rel_map)
#     print_summary(pose_park_rel_map, pose_park_start_rel_map, distance_to_park, point_to_park_angle, orient_with_park_angle)
#
#     assert_almost_equal(distance_to_park, math.sqrt(2), 3)
#     assert_almost_equal(point_to_park_angle, 0.785398163397, 3)
#     assert_almost_equal(orient_with_park_angle, 3.14159265359-0.785398163397)
#
#     # 2
#     pose_park_start_rel_map = geometry_msgs.Pose()
#     pose_park_start_rel_map.position = geometry_msgs.Point(1.0, 0, 0)
#     pose_park_start_rel_map.orientation = geometry_msgs.Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
#     pose_park_rel_map = geometry_msgs.Pose()
#     pose_park_rel_map.position = geometry_msgs.Point(1.573153, -0.52934, 0.0)
#     pose_park_rel_map.orientation = geometry_msgs.Quaternion(x=0.0, y=0.0, z=-0.370314263041, w=0.928906532751)
#
#     (distance_to_park, point_to_park_angle, orient_with_park_angle) = park.compute_parking_geometry(pose_park_start_rel_map, pose_park_rel_map)
#     print_summary(pose_park_rel_map, pose_park_start_rel_map, distance_to_park, point_to_park_angle, orient_with_park_angle)
#
#     assert_almost_equal(distance_to_park, 0.780195614579, 3)
#     assert_almost_equal(point_to_park_angle, -0.74567912866881458, 3)
#     assert_almost_equal(orient_with_park_angle, -0.013015497172437551, 3)

def test_concatenate_poses():
    print(console.bold + "\n****************************************************************************************" + console.reset)
    print(console.bold + "* Concatenate Poses " + console.reset)
    print(console.bold + "****************************************************************************************\n" + console.reset)

    pose_goal_rel_map = geometry_msgs.Pose()
    pose_goal_rel_map.position = geometry_msgs.Point(0.5, 0, 0)
    pose_goal_rel_map.orientation = geometry_msgs.Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

    pose_map_rel_odom = geometry_msgs.Pose()
    pose_map_rel_odom.position = geometry_msgs.Point(-0.771929405738, -0.333180496296, 0)
    pose_map_rel_odom.orientation = geometry_msgs.Quaternion(x=0.0, y=0.0, z=0.71485625728, w=0.699271429009)  # 90 degrees

    pose_goal_rel_odom = transform_utilities.concatenate_poses(pose_goal_rel_map, pose_map_rel_odom)
    print("Goal Rel Map: %s" % pose_goal_rel_map)
    print("Map Rel Odom: %s" % pose_map_rel_odom)
    print("Goal Rel Odom: %s" % pose_goal_rel_odom)

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
import gopher_behaviours.transform_utilities as transform_utilities
import math
import os
import rocon_console.console as console


##############################################################################
# Tests
##############################################################################

def test_pose_start_rel_finish():
    print(console.bold + "\n****************************************************************************************" + console.reset)
    print(console.bold + "* get_relative_pose " + console.reset)
    print(console.bold + "****************************************************************************************" + console.reset)
    print(console.green + "1m Forward" + console.reset)
    start_pose = geometry_msgs.Pose()
    start_pose.position = geometry_msgs.Point(0.0, 0.0, 0.0)
    start_pose.orientation = geometry_msgs.Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    finish_pose = geometry_msgs.Pose()
    finish_pose.position = geometry_msgs.Point(1.0, 0.0, 0.0)
    finish_pose.orientation = geometry_msgs.Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    pose_start_rel_finish = transform_utilities.get_relative_pose(start_pose, finish_pose)
    pose_finish_rel_start = transform_utilities.get_relative_pose(finish_pose, start_pose)
    print("%s" % pose_finish_rel_start)

    print(console.green + "1m Forward + 90 degrees" + console.reset)
    start_pose = geometry_msgs.Pose()
    start_pose.position = geometry_msgs.Point(0.0, 0.0, 0.0)
    start_pose.orientation = geometry_msgs.Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    finish_pose = geometry_msgs.Pose()
    finish_pose.position = geometry_msgs.Point(1.0, 0.0, 0.0)
    finish_pose.orientation = geometry_msgs.Quaternion(x=0.0, y=0.0, z=1.0/math.sqrt(2), w=1.0/math.sqrt(2))
    pose_start_rel_finish = transform_utilities.get_relative_pose(start_pose, finish_pose)
    pose_finish_rel_start = transform_utilities.get_relative_pose(finish_pose, start_pose)
    print("%s" % pose_finish_rel_start)

    print(console.green + "1m Forward + 180 degrees from non-zero starting location" + console.reset)
    start_pose = geometry_msgs.Pose()
    start_pose.position = geometry_msgs.Point(4.0, 3.0, 0.0)
    start_pose.orientation = geometry_msgs.Quaternion(x=0.0, y=0.0, z=-1.0/math.sqrt(2), w=1.0/math.sqrt(2))
    finish_pose = geometry_msgs.Pose()
    finish_pose.position = geometry_msgs.Point(5.0, 3.0, 0.0)
    finish_pose.orientation = geometry_msgs.Quaternion(x=0.0, y=0.0, z=1.0/math.sqrt(2), w=1.0/math.sqrt(2))
    pose_start_rel_finish = transform_utilities.get_relative_pose(start_pose, finish_pose)
    pose_finish_rel_start = transform_utilities.get_relative_pose(finish_pose, start_pose)
    print("%s" % pose_finish_rel_start)

    print(console.bold + "\n****************************************************************************************" + console.reset)
    print(console.bold + "* angle_between " + console.reset)
    print(console.bold + "****************************************************************************************" + console.reset)

    vector_x = [1, 0, 0]
    vector_90 = [0, 1, 0]
    vector_135 = [-1, 1, 0]
    vector_180 = [-1, 0, 0]
    vector_225 = [-1, -1, 0]
    vector_270 = [0, -1, 0]
    vector_315 = [1, -1, 0]
    angle_90_to_x = transform_utilities.angle_between(vector_90, vector_x)
    angle_x_to_90 = transform_utilities.angle_between(vector_x, vector_90)
    angle_x_to_135 = transform_utilities.angle_between(vector_x, vector_135)
    angle_x_to_180 = transform_utilities.angle_between(vector_x, vector_180)
    angle_x_to_225 = transform_utilities.angle_between(vector_x, vector_225)
    angle_x_to_270 = transform_utilities.angle_between(vector_x, vector_270)
    angle_x_to_315 = transform_utilities.angle_between(vector_x, vector_315)
    print("-90: %s" % angle_90_to_x)
    assert_almost_equal(angle_90_to_x, -1.57079632679)
    print(" 90: %s"  % angle_x_to_90)
    assert_almost_equal(angle_x_to_90, 1.57079632679)
    print("135: %s" % angle_x_to_135)
    assert_almost_equal(angle_x_to_135, 2.35619449019)
    print("180: %s" % angle_x_to_180)
    assert_almost_equal(angle_x_to_180, 3.14159265359)
    print("225: %s" % angle_x_to_225)
    assert_almost_equal(angle_x_to_225, -2.35619449019)
    print("270: %s" % angle_x_to_270)
    assert_almost_equal(angle_x_to_270, -1.57079632679)
    print("315: %s" % angle_x_to_315)
    assert_almost_equal(angle_x_to_315, -0.785398163397)

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

from nose.tools import assert_raises
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
    print(console.bold + "* pose_finish_rel_start " + console.reset)
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

#     assert(isinstance(topological_path[0], gopher_semantic_msgs.Location))
#     assert(isinstance(topological_path[1], gopher_semantic_msgs.Location))
#     assert(isinstance(topological_path[2], gopher_semantic_msgs.Elevator))
#     assert(isinstance(topological_path[3], gopher_semantic_msgs.Location))


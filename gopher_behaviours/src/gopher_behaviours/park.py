#!/usr/bin/env python
#
# License: Yujin
#
##############################################################################
# Description
##############################################################################

"""
.. module:: park
   :platform: Unix
   :synopsis: The parking logic.

Oh my spaghettified magnificence,
Bless my noggin with a tickle from your noodly appendages!
"""

##############################################################################
# Imports
##############################################################################

import gopher_configuration
import gopher_std_msgs.msg as gopher_std_msgs
import navigation
import numpy
import py_trees

from . import ar_markers
from . import battery
from . import docking
from . import simple_motions
from . import transform_utilities

##############################################################################
# Behaviours
##############################################################################


class Park(py_trees.Sequence):
    """
    Blackboard Variables:

     - pose_park_rel_map       (r)  [geometry_msgs/Pose]                     : pose of the parking location relative to the homebase
     - pose_park_start_rel_map (w)  [geometry_msgs.PoseWithCovarianceStamped]: transferred from /navi/pose when about to park
     - undocked                (rc) [bool]                                   : reads to determine if it should dock, and clears afterwards
    """
    def __init__(self, name="Park"):
        super(Park, self).__init__(name)
        self.gopher = gopher_configuration.Configuration(fallback_to_defaults=True)
        self.blackbox_level = py_trees.common.BlackBoxLevel.BIG_PICTURE

        ############################################
        # Behaviours
        ############################################
        check_didnt_undock = py_trees.meta.inverter(py_trees.CheckBlackboardVariable)(
            name='Didnt Undock',
            variable_name='undocked',
            expected_value=True
        )
        write_pose_park_start_rel_map = navigation.create_map_pose_to_blackboard_behaviour(
            name="Start Pose (Map)",
            blackboard_variables={"pose_park_start_rel_map": None}
        )
        parking_motions = Approach()
        todock_or_not_todock = py_trees.Selector(name="ToDock or Not ToDock")
        docking_control = py_trees.Sequence(name="Dock")
        docking_control.blackbox_level = py_trees.common.BlackBoxLevel.COMPONENT
        pre_dock_rotation = simple_motions.SimpleMotion(
            name="Pre-Rotation",
            motion_type=gopher_std_msgs.SimpleMotionGoal.MOTION_ROTATE,
            motion_amount=3.14
        )
        (ar_tracker_on, ar_tracker_off) = ar_markers.create_ar_tracker_pair_blackboxes()
        docking_controller = docking.DockingController(name="Docking Controller")
        clearing_flags = py_trees.blackboard.ClearBlackboardVariable(name="Clear Flags", variable_name="undocked")

        ############################################
        # Assembly
        ############################################
        self.add_child(write_pose_park_start_rel_map)
        self.add_child(parking_motions)
        self.add_child(todock_or_not_todock)
        todock_or_not_todock.add_child(check_didnt_undock)
        todock_or_not_todock.add_child(docking_control)
        docking_control.add_child(pre_dock_rotation)
        docking_control.add_child(ar_tracker_on)
        docking_control.add_child(docking_controller)
        docking_control.add_child(ar_tracker_off)
        self.add_child(clearing_flags)

    @classmethod
    def render_dot_tree(cls):
        """
        Render the subtree to file.
        """
        root = cls()
        py_trees.display.render_dot_tree(root)


def compute_parking_geometry(pose_park_start_rel_map, pose_park_rel_map, distance_threshold=0.5):
    """
    Helper function for computing the parking geometry from the starting pose and the stored
    parking pose (both relative to map).

    :param geometry_msgs.Pose pose_park_start_rel_map: starting pose for the maneuvre
    :param geometry_msgs.Pose pose_park_rel_map      : saved parking location
    :param float distance_threshold: 'close enough' parameter with respect to translations

    :returns: 3-tuple of distance_to_park, point_to_park_angle, orient_with_park_angle

    The return angles are the rotation amounts that are required for each step of a
    rotate - translate - rotate simple motions maneuvre.

    If the current pose is closer than the distance threshold from the parking pose, then
    it will drop back to computing for a one step simple motions maneuvre and return
    (0.0 - 0.0 - angle) 3-tuple.
    """
    # init some variables
    pose_park_rel_start = transform_utilities.get_relative_pose(pose_park_rel_map, pose_park_start_rel_map)
    current_pose_angle = transform_utilities.angle_from_geometry_msgs_quaternion(pose_park_start_rel_map.orientation)
    park_pose_angle = transform_utilities.angle_from_geometry_msgs_quaternion(pose_park_rel_map.orientation)
    # how far to go?
    distance_to_park = transform_utilities.norm_from_geometry_msgs_pose(pose_park_rel_start)

    # 1-step or 3-step?
    if distance_to_park < distance_threshold:
        distance_to_park = 0.0
        point_to_park_angle = 0.0
        orient_with_park_angle = park_pose_angle - current_pose_angle
    else:
        translation_park_rel_start = [
            pose_park_rel_map.position.x - pose_park_start_rel_map.position.x,
            pose_park_rel_map.position.y - pose_park_start_rel_map.position.y,
            pose_park_rel_map.position.z - pose_park_start_rel_map.position.z
        ]
        direction_to_park_angle = transform_utilities.angle_between([1.0, 0.0, 0.0], translation_park_rel_start)
        point_to_park_angle = direction_to_park_angle - current_pose_angle
        orient_with_park_angle = park_pose_angle - current_pose_angle - point_to_park_angle

    # wrap to -pi, pi
    point_to_park_angle = (point_to_park_angle + numpy.pi) % (2 * numpy.pi) - numpy.pi
    orient_with_park_angle = (orient_with_park_angle + numpy.pi) % (2 * numpy.pi) - numpy.pi

    return (distance_to_park, point_to_park_angle, orient_with_park_angle)


class Approach(py_trees.Sequence):
    """
    Dynamically stitches together some simple motions on the fly (on entry via
    the initialise method with data from the blackboard) and adds them as children.
    These children are then dumped when the sequence is stopped or invalidated.

    Blackboard Variables:

     - pose_park_rel_map       (r) [geometry_msgs/Pose]                     : pose of the parking location relative to the homebase
     - pose_park_start_rel_map (w) [geometry_msgs.PoseWithCovarianceStamped]: transferred from /navi/pose when about to park
    """

    def __init__(self, name="Approach"):
        super(Approach, self).__init__(name)
        self.blackbox_level = py_trees.common.BlackBoxLevel.COMPONENT

        self.close_to_park_distance_threshold = 0.1
        self.blackboard = py_trees.Blackboard()

        # dummy children for dot graph rendering purposes
        # they will get cleaned up the first time this enters initialise
        point_to_park = simple_motions.SimpleMotion(
            name="Point to Park (?rad)",
            motion_type=gopher_std_msgs.SimpleMotionGoal.MOTION_ROTATE,
            motion_amount=0.0,
        )
        move_to_park = simple_motions.SimpleMotion(
            name="Move to Park (?m)",
            motion_type=gopher_std_msgs.SimpleMotionGoal.MOTION_TRANSLATE,
            motion_amount=0.0,
        )
        orient_with_park = simple_motions.SimpleMotion(
            name="Orient to Park (?rad)",
            motion_type=gopher_std_msgs.SimpleMotionGoal.MOTION_ROTATE,
            motion_amount=0.0,
        )
        self.add_child(point_to_park)
        self.add_child(move_to_park)
        self.add_child(orient_with_park)

    def initialise(self):
        # cleanup the old children
        self.remove_all_children()

        pose_park_rel_map = self.blackboard.pose_park_rel_map
        pose_park_start_rel_map = self.blackboard.pose_park_start_rel_map.pose.pose  # get the geometry_msgs.Pose object

        # do some trig
        distance_to_park, point_to_park_angle, orient_with_park_angle = compute_parking_geometry(
            pose_park_start_rel_map,
            pose_park_rel_map,
            self.close_to_park_distance_threshold
        )

        self.blackboard.distance_to_park = distance_to_park
        self.blackboard.point_to_park_angle = point_to_park_angle
        self.blackboard.orient_with_park_angle = orient_with_park_angle

        epsilon = 0.01
        if abs(point_to_park_angle) > epsilon:
            point_to_park = simple_motions.SimpleMotion(
                name="Point to Park %0.2f rad" % point_to_park_angle,
                motion_type=gopher_std_msgs.SimpleMotionGoal.MOTION_ROTATE,
                motion_amount=point_to_park_angle,
                live_dangerously=True
            )
            self.add_child(point_to_park)
        if abs(distance_to_park) > epsilon:
            move_to_park = simple_motions.SimpleMotion(
                name="Move to Park %0.2f m" % distance_to_park,
                motion_type=gopher_std_msgs.SimpleMotionGoal.MOTION_TRANSLATE,
                motion_amount=distance_to_park,
                live_dangerously=True
            )
            self.add_child(move_to_park)
        if abs(orient_with_park_angle) > epsilon:
            orient_with_park = simple_motions.SimpleMotion(
                name="Orient to Park %0.2f rad" % orient_with_park_angle,
                motion_type=gopher_std_msgs.SimpleMotionGoal.MOTION_ROTATE,
                motion_amount=orient_with_park_angle,
                live_dangerously=True
            )
            self.add_child(orient_with_park)

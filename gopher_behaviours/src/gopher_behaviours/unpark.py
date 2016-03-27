#!/usr/bin/env python
#
# License: Yujin
#
##############################################################################
# Description
##############################################################################
"""
.. module:: unpark
   :platform: Unix
   :synopsis: The unparking logic.

Oh my spaghettified magnificence,
Bless my noggin with a tickle from your noodly appendages!

----

"""

##############################################################################
# Imports
##############################################################################

import elf_msgs.msg as elf_msgs
from gopher_semantics.semantics import Semantics
import geometry_msgs.msg as geometry_msgs
import gopher_std_msgs.msg as gopher_std_msgs
import gopher_std_msgs.srv as gopher_std_srvs
import gopher_configuration
import py_trees
import rospy
import tf

from . import ar_markers
from . import battery
from . import docking
from . import interactions
from . import navigation
from . import transform_utilities

##############################################################################
# Implementation
##############################################################################


class UnPark(py_trees.Sequence):
    """
    Unpark the robot from its parking spot

    Blackboard Variables:

     - auto_init_failed            (r) [bool]                       : written by the auto-init program, use it to trigger a manual initialisation
     - elf_localisation_status     (w) [elf_msgs/ElfLocaliserStatus]: the localisation status (gathered at the start of this behaviour)
     - event_stop_button           (r) [bool]                       : written onto the blackboard by an event handler, catch it for cancelling purposes
     - homebase                    (w) [custom dict]                : semantic information about the homebase
     - pose_homebase_rel_map       (w) [geometry_msgs/Pose]         : pose of the homebase relative to the map, obtained from semantics
     - pose_unpark_start_rel_map   (w) [geometry_msgs/Pose]         : starting park location when already localised, transferred from /navi/pose
     - pose_unpark_start_rel_odom  (w) [geometry_msgs/Pose]         : starting park location when not yet localised, transferred from /gopher/odom
     - pose_unpark_finish_rel_odom (w) [geometry_msgs/Pose]         : finishing park location when not yet localised, transferred from /gopher/odom
     - pose_unpark_finish_rel_map  (w) [geometry_msgs/Pose]         : finishing park location after having observed it is localised, transferred from /navi/pose
     - pose_park_rel_homebase      (w) [geometry_msgs/Pose]         : pose of the parking location relative to the homebase computed from start/finish poses
     - pose_park_rel_map           (w) [geometry_msgs/Pose]         : pose of the parking location relative to the homebase, computed from pose_park_rel_homebase and pose_homebase_rel_map
    """
    def __init__(self, name="unpark"):
        super(UnPark, self).__init__(name)
        # fallback to defaults so we don't require the ros param server to do dot graphs, etc.
        self.gopher = gopher_configuration.Configuration(fallback_to_defaults=True)
        self.semantic_locations = None
        self.blackboard = py_trees.Blackboard()

        ################################################################
        # Flags
        ################################################################
        init_flags = py_trees.blackboard.SetBlackboardVariable(name="Init Flags", variable_name="undocked", variable_value=False)
        set_docked_flag = py_trees.blackboard.SetBlackboardVariable(name="Flag Docked", variable_name="undocked", variable_value=True)

        ################################################################
        # Undock
        ################################################################
        unplug_undock = py_trees.Selector(name="UnPlug/UnDock")
        ar_markers_on = ar_markers.ControlARMarkerTracker("AR Markers On", self.gopher.topics.ar_tracker_long_range, True)
        ar_markers_off = ar_markers.ControlARMarkerTracker("AR Markers Off", self.gopher.topics.ar_tracker_long_range, False)
        undocking = py_trees.Sequence(name="UnDock")
        break_out = py_trees.meta.failure_is_success(
            navigation.SimpleMotion(
                name="Break Out",
                motion_type=gopher_std_msgs.SimpleMotionGoal.MOTION_TRANSLATE,
                motion_amount=0.5
            )
        )
        is_docked = battery.create_is_docked(name="Is Docked?")
        is_discharging = battery.create_is_discharging(name="Is Discharging?")
        unplug = py_trees.Selector(name="UnPlug")
        wait_to_be_unplugged = battery.create_wait_to_be_unplugged(name="Flash for Help")
        auto_undock = docking.DockingController(name="Auto UnDock", undock=True)

        ################################################################
        # Unplug and Pre-Localising
        ################################################################
        write_localisation_status = navigation.create_elf_localisation_to_blackboard_behaviour(name="Get Localisation Status", blackboard_variables={"elf_localisation_status": "status"})
        path_chooser = py_trees.Selector("Localised?")

        ################################################################
        # Already Localised Sequence Components
        ################################################################
        # TODO : check if we significantly moved from the saved location

        already_localised_sequence = py_trees.Sequence("Already Localised")
        are_we_localised = py_trees.CheckBlackboardVariable(
            name="Check ELF Status",
            variable_name="elf_localisation_status",
            expected_value=elf_msgs.ElfLocaliserStatus.STATUS_WORKING
        )
        write_starting_pose_from_map = navigation.create_map_pose_to_blackboard_behaviour(
            name="Start Pose (Map)",
            blackboard_variables={"pose_unpark_start_rel_map": "pose.pose"}
        )
        update_parking_pose_from_map = UpdateParkingPoseFromMap(name="Update Park Pose (Map)")

        ################################################################
        # Not Localised Sequence Components
        ################################################################

        not_yet_localised_sequence = py_trees.Sequence("Not Localised")
        manual_unpark = py_trees.Sequence("Manual")
        automatic_unpark = py_trees.Sequence("Automatic")
        manual_or_auto = py_trees.Selector("Initialisation")

        ##############################
        # Common
        ##############################
        write_starting_pose_from_odom = navigation.create_odom_pose_to_blackboard_behaviour(name="Start Pose (Odom)", blackboard_variables={"pose_unpark_start_rel_odom": "pose.pose"})

        ##############################
        # Manual
        ##############################
        manual_write_finishing_pose_from_odom = navigation.create_odom_pose_to_blackboard_behaviour(name="Final Pose (Odom)", blackboard_variables={"pose_unpark_finish_rel_odom": "pose.pose"})
        manual_checks = py_trees.composites.Selector("Manual Checks")
        is_cancel_activated = py_trees.CheckBlackboardVariable(
            name='Is Cancelled?',
            variable_name='event_stop_button',
            expected_value=True
        )
        interactions.create_check_for_stop_button_press('Is Cancelled?')
        did_auto_init_fail = py_trees.composites.Sequence("Auto Init Timed Out?")
        check_auto_init_flag = py_trees.CheckBlackboardVariable(
            name='Check Flag',
            variable_name='auto_init_failed',
            expected_value=True
        )
        clear_auto_init_flag = py_trees.blackboard.ClearBlackboardVariable(
            name="Clear Flag",
            variable_name='auto_init_failed'
        )

        wait_for_go_button_press = interactions.create_wait_for_go_button("Wait for Go Button")

        teleport = navigation.create_homebase_teleport()
        go_to_homebase = interactions.SendNotification('Teleop to Homebase',
                                                       message='waiting for button press to continue',
                                                       led_pattern=self.gopher.led_patterns.humans_i_need_help,
                                                       duration=gopher_std_srvs.NotifyRequest.INDEFINITE)
        go_to_homebase.add_child(wait_for_go_button_press)
        manual_save_parking_pose = SaveParkingPoseManual("Save Park Pose (Manual)")

        ##############################
        # Automatic
        ##############################
        auto_initialisation = navigation.ElfInitialisation(name="Elf Initialisation")
        auto_write_finishing_pose_from_odom = navigation.create_odom_pose_to_blackboard_behaviour(name="Final Pose (Odom)", blackboard_variables={"pose_unpark_finish_rel_odom": "pose.pose"})
        auto_write_finishing_pose_from_map = navigation.create_elf_pose_to_blackboard_behaviour(name="Finishing Pose (Map)", blackboard_variables={"pose_unpark_finish_rel_map": "pose.pose"})
        auto_save_park_pose = SaveParkingPoseAuto("Save Park Pose (Auto)")

        ################################################################
        # All Together
        ################################################################

        self.add_child(init_flags)
        self.add_child(unplug_undock)
        unplug_undock.add_child(undocking)
        undocking.add_child(is_docked)
        undocking.add_child(ar_markers_on)
        undocking.add_child(auto_undock)
        undocking.add_child(ar_markers_off)
        undocking.add_child(break_out)
        undocking.add_child(set_docked_flag)
        unplug_undock.add_child(unplug)
        unplug.add_child(is_discharging)
        unplug.add_child(wait_to_be_unplugged)
        self.add_child(write_localisation_status)
        self.add_child(path_chooser)
        path_chooser.add_child(already_localised_sequence)
        already_localised_sequence.add_child(are_we_localised)
        already_localised_sequence.add_child(write_starting_pose_from_map)
        already_localised_sequence.add_child(update_parking_pose_from_map)
        path_chooser.add_child(not_yet_localised_sequence)
        not_yet_localised_sequence.add_child(write_starting_pose_from_odom)
        not_yet_localised_sequence.add_child(manual_or_auto)
        manual_or_auto.add_child(manual_unpark)
        manual_unpark.add_child(manual_checks)
        manual_checks.add_child(is_cancel_activated)
        manual_checks.add_child(did_auto_init_fail)
        did_auto_init_fail.add_child(check_auto_init_flag)
        did_auto_init_fail.add_child(clear_auto_init_flag)
        manual_unpark.add_child(go_to_homebase)
        manual_unpark.add_child(teleport)
        manual_unpark.add_child(manual_write_finishing_pose_from_odom)
        manual_unpark.add_child(manual_save_parking_pose)
        manual_or_auto.add_child(automatic_unpark)
        automatic_unpark.add_child(auto_initialisation)
        automatic_unpark.add_child(auto_write_finishing_pose_from_odom)
        automatic_unpark.add_child(auto_write_finishing_pose_from_map)
        automatic_unpark.add_child(auto_save_park_pose)

    def setup(self, timeout):
        self.logger.debug("  %s [UnPark::setup()]" % self.name)
        if not hasattr(self.blackboard, 'homebase_translation'):
            semantic_locations = Semantics(self.gopher.namespaces.semantics)
            self.blackboard.homebase = semantic_locations.semantic_modules['locations']['homebase']
            # assume map frame is 0,0,0 with no rotation; translation of homebase is
            # the position of the homebase specified in semantic locations. We need
            # the homebase location regardless of whether dslam is initialised.
            position = geometry_msgs.Point(self.blackboard.homebase.pose.x, self.blackboard.homebase.pose.y, 0)
            q = tf.transformations.quaternion_about_axis(self.blackboard.homebase.pose.theta, (0, 0, 1))  # returns a []
            orientation = geometry_msgs.Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
            self.blackboard.pose_homebase_rel_map = geometry_msgs.Pose(position=position, orientation=orientation)
        return py_trees.Sequence.setup(self, timeout)

    @classmethod
    def render_dot_tree(cls):
        """
        Render the subtree to file.
        """
        root = cls()
        py_trees.display.render_dot_tree(root)


class UpdateParkingPoseFromMap(py_trees.Behaviour):
    """
    This will compare the starting pose (rel map) with the currently saved parking pose. If there's a
    significant difference, it will replace the existing parking pose in the blackboard.

    Blackboard Variables:

     - pose_unpark_start_rel_map   (w) [geometry_msgs/Pose] : starting park location when already localised, transferred from /navi/pose
     - pose_park_rel_map          (rw) [geometry_msgs/Pose] : pre-existing pose of the parking location relative to the homebase
    """
    def __init__(self, name="Update Parking Pose"):
        super(UpdateParkingPoseFromMap, self).__init__(name)
        self.blackboard = py_trees.Blackboard()
        self.close_to_park_distance_threshold = 0.25

    def update(self):
        """
        Grab the difference between previously stored and currently retrieved parking locations
        and if there is a significant difference, update it.
        """
        # First check that a previous location was stored - this can happen if we are first tick
        # through the unparking tree and the navigation system happened to be already localised
        if not hasattr(self.blackboard, "pose_park_rel_map"):
            rospy.loginfo("Behaviours [%s]: new parking location identified, writing." % self.name)
            self.blackboard.pose_park_rel_map = self.blackboard.pose_unpark_start_rel_map
            return py_trees.Status.SUCCESS

        # Typical case - we already registered a park pose from previous ticks through the behaviour tree
        pose_start_rel_park = transform_utilities.get_relative_pose(
            self.blackboard.pose_unpark_start_rel_map,
            self.blackboard.pose_park_rel_map
        )
        distance = transform_utilities.norm_from_geometry_msgs_pose(pose_start_rel_park)
        if distance > self.close_to_park_distance_threshold:
            rospy.loginfo("Behaviours [%s]: parking location shifted, updating." % self.name)
            self.blackboard.pose_park_rel_map = self.blackboard.pose_unpark_start_rel_map
        return py_trees.Status.SUCCESS


class SaveParkingPoseManual(py_trees.Behaviour):
    """
    Takes the two odom readings and stitches them together to form pose_park_rel_homebase.
    Not worrying about generalising this operation here, it is a worker for the UnPark blackbox.

    This is triggered when doing a manual teleop to the homebase and we have no guarantee of
    localisation at start or finish.

    Blackboard Variables:

     - pose_homebase_rel_map       (r) [geometry_msgs/Pose]     : pose of the homebase relative to the map, obtained from semantics
     - pose_unpark_start_rel_odom  (r) [geometry_msgs/Pose]     : starting park location when not yet localised, transferred from /gopher/odom
     - pose_unpark_finish_rel_odom (r) [geometry_msgs/Pose]     : finishing park location when not yet localised, transferred from /gopher/odom
     - pose_park_rel_homebase      (w) [geometry_msgs/Pose]     : pose of the parking location relative to the homebase
     - pose_park_rel_map           (w) [geometry_msgs/Pose]     : pose of the parking location relative to the homebase, computed from pose_park_rel_homebase and pose_homebase_rel_map
    """
    def __init__(self, name="Save Parking Pose"):
        super(SaveParkingPoseManual, self).__init__(name)
        self.blackboard = py_trees.Blackboard()

    def update(self):
        geometry_msgs_pose_start_rel_odom = self.blackboard.pose_unpark_start_rel_odom
        geometry_msgs_pose_finish_rel_odom = self.blackboard.pose_unpark_finish_rel_odom

        pose_start_rel_finish = transform_utilities.get_relative_pose(
            geometry_msgs_pose_start_rel_odom,
            geometry_msgs_pose_finish_rel_odom
        )
        self.blackboard.pose_park_rel_homebase = pose_start_rel_finish
        self.blackboard.pose_park_rel_map = transform_utilities.concatenate_poses(
            self.blackboard.pose_park_rel_homebase,
            self.blackboard.pose_homebase_rel_map
        )
        return py_trees.Status.SUCCESS


class SaveParkingPoseAuto(py_trees.Behaviour):
    """
    Takes the start and final odom readings along with the newly localised map pose and combines them:

    - pose_park_rel_map = pose_unpark_start_rel_finish + pose_current_rel_map

    Note, here start~park, finish~current.

    Blackboard Variables:

     - pose_homebase_rel_map       (r) [geometry_msgs/Pose]     : pose of the homebase relative to the map, obtained from semantics
     - pose_unpark_start_rel_odom  (r) [geometry_msgs/Pose]     : starting park location when not yet localised, transferred from /gopher/odom
     - pose_unpark_finish_rel_map  (r) [geometry_msgs/Pose]     : finishing park location after having observed it is localised, transferred from /navi/pose
     - pose_park_rel_map           (w) [geometry_msgs/Pose]     : pose of the parking location relative to the homebase, computed from pose_park_rel_homebase and pose_homebase_rel_map
    """
    def __init__(self, name="Save Parking Pose"):
        super(SaveParkingPoseAuto, self).__init__(name)
        self.blackboard = py_trees.Blackboard()

    def update(self):
        geometry_msgs_pose_start_rel_odom = self.blackboard.pose_unpark_start_rel_odom
        geometry_msgs_pose_finish_rel_odom = self.blackboard.pose_unpark_finish_rel_odom

        pose_start_rel_finish = transform_utilities.get_relative_pose(
            geometry_msgs_pose_start_rel_odom,
            geometry_msgs_pose_finish_rel_odom
        )
        self.blackboard.pose_park_rel_map = transform_utilities.concatenate_poses(
            pose_start_rel_finish,
            self.blackboard.pose_unpark_finish_rel_map  # pose_homebase_rel_map
        )
        # for completeness we could compute this from park_rel_map and homebase_rel_map, but do we need to?
        # self.blackboard.pose_park_rel_homebase =

        return py_trees.Status.SUCCESS

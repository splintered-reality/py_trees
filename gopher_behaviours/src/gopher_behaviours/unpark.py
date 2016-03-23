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
import gopher_configuration
import py_trees
import rospy
import tf

from . import battery
from . import navigation
from . import starting
from . import transform_utilities
from .interactions import WaitForButton, CheckButtonPressed, SendNotification

##############################################################################
# Implementation
##############################################################################


class UnPark(py_trees.Sequence):
    """
    Unpark the robot from its parking spot

    Blackboard Variables:

     - elf_localisation_status     (w) [elf_msgs/ElfLocaliserStatus]: the localisation status (gathered at the start of this behaviour)
     - homebase                    (w) [custom dict]                : semantic information about the homebase
     - pose_homebase_rel_map       (w) [geometry_msgs/Pose]         : pose of the homebase relative to the map, obtained from semantics
     - pose_unpark_start_rel_map   (w) [geometry_msgs/Pose]         : starting park location when already localised, transferred from /navi/pose
     - pose_unpark_start_rel_odom  (w) [geometry_msgs/Pose]         : starting park location when not yet localised, transferred from /gopher/odom
     - pose_unpark_finish_rel_odom (w) [geometry_msgs/Pose]         : finishing park location when not yet localised, transferred from /gopher/odom
     - pose_park_rel_homebase      (w) [geometry_msgs/Pose]         : pose of the parking location relative to the homebase computed from start/finish poses
     - pose_park_rel_map           (w) [geometry_msgs/Pose]         : pose of the parking location relative to the homebase, computed from pose_park_rel_homebase and pose_homebase_rel_map
     - last_starting_action        (w) [starting.StartingAction]    : last starting action gets set to unparked
    """
    def __init__(self, name="unpark"):
        super(UnPark, self).__init__(name)
        # fallback to defaults so we don't require the ros param server to do dot graphs, etc.
        self.gopher = gopher_configuration.Configuration(fallback_to_defaults=True)
        self.semantic_locations = None
        self.blackboard = py_trees.Blackboard()

        ################################################################
        # Preliminary Guards & Data Gathering
        ################################################################

        wait_to_be_unplugged = SendNotification('UnPlug Me', message='waiting to be unplugged', led_pattern=self.gopher.led_patterns.humans_i_need_help)
        discharging = battery.create_check_discharging_behaviour()
        wait_to_be_unplugged_condition_one = py_trees.behaviours.Condition('Wait to be Unplugged', child=discharging, succeed_status=py_trees.Status.SUCCESS)
        wait_to_be_unplugged.add_child(wait_to_be_unplugged_condition_one)
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
        manual_unpark = py_trees.Sequence("Manual UnPark")
        automatic_unpark = py_trees.Sequence("Automatic UnPark")
        manual_or_auto = py_trees.Selector("Manual or Auto")

        ##############################
        # Common
        ##############################
        write_starting_pose_from_odom = navigation.create_odom_pose_to_blackboard_behaviour(name="Start Pose (Odom)", blackboard_variables={"pose_unpark_start_rel_odom": "pose.pose"})

        ##############################
        # Manual
        ##############################
        manual_write_finishing_pose_from_odom = navigation.create_odom_pose_to_blackboard_behaviour(name="Final Pose (Odom)", blackboard_variables={"pose_unpark_finish_rel_odom": "pose.pose"})
        is_cancel_activated = CheckButtonPressed('Cancel Activated?', self.gopher.buttons.cancel, latched=True)
        wait_for_go_button_press = WaitForButton('Wait for Go Button', self.gopher.buttons.go)
        teleport = navigation.create_homebase_teleport()
        go_to_homebase = SendNotification('Teleop to Homebase', message='waiting for button press to continue', led_pattern=self.gopher.led_patterns.humans_i_need_help)
        go_to_homebase.add_child(wait_for_go_button_press)
        update_parking_pose_from_odom = SaveParkingPoseFromOdom("Save Park Pose (Manual)")

        ##############################
        # Automatic
        ##############################
        auto_initialisation = navigation.ElfInitialisation(name="Elf Initialisation")
        auto_write_finishing_pose_from_odom = navigation.create_odom_pose_to_blackboard_behaviour(name="Final Pose (Odom)", blackboard_variables={"pose_unpark_finish_rel_odom": "pose.pose"})
        auto_write_finishing_pose_from_map = navigation.create_map_pose_to_blackboard_behaviour(name="Finishing Pose (Map)", blackboard_variables={"pose_unpark_finish_rel_map": "pose.pose"})
        auto_update_parking_pose_from_odom_and_map = py_trees.behaviours.Success("Save Park Pose (Auto)")

        ################################################################
        # All Together
        ################################################################

        self.add_child(wait_to_be_unplugged)
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
        manual_unpark.add_child(is_cancel_activated)
        manual_unpark.add_child(go_to_homebase)
        manual_unpark.add_child(teleport)
        manual_unpark.add_child(manual_write_finishing_pose_from_odom)
        manual_unpark.add_child(update_parking_pose_from_odom)
        manual_or_auto.add_child(automatic_unpark)
        automatic_unpark.add_child(auto_initialisation)
        automatic_unpark.add_child(auto_write_finishing_pose_from_odom)
        automatic_unpark.add_child(auto_write_finishing_pose_from_map)
        automatic_unpark.add_child(auto_update_parking_pose_from_odom_and_map)

    def initialise(self):
        py_trees.Sequence.initialise(self)
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
        pose_start_rel_park = transform_utilities.get_relative_pose(
            self.blackboard.pose_unpark_start_rel_map,
            self.blackboard.pose_park_rel_map
        )
        distance = transform_utilities.norm_from_geometry_msgs_pose(pose_start_rel_park)
        if distance > self.close_to_park_distance_threshold:
            rospy.loginfo("Behaviours [%s]: parking location shifted, updating." % self.name)
            self.blackboard.pose_park_rel_map = self.blackboard.pose_unpark_start_rel_map
        return py_trees.Status.SUCCESS


class SaveParkingPoseFromOdom(py_trees.Behaviour):
    """
    Takes the two odom readings and stitches them together to form pose_park_rel_homebase.
    Not worrying about generalising this operation here, it is a worker for the UnPark blackbox.

    Blackboard Variables:

     - pose_homebase_rel_map       (r) [geometry_msgs/Pose]     : pose of the homebase relative to the map, obtained from semantics
     - pose_unpark_start_rel_odom  (w) [geometry_msgs/Pose]     : starting park location when not yet localised, transferred from /gopher/odom
     - pose_unpark_finish_rel_odom (w) [geometry_msgs/Pose]     : finishing park location when not yet localised, transferred from /gopher/odom
     - pose_park_rel_homebase      (w) [geometry_msgs/Pose]     : pose of the parking location relative to the homebase
     - pose_park_rel_map           (w) [geometry_msgs/Pose]     : pose of the parking location relative to the homebase, computed from pose_park_rel_homebase and pose_homebase_rel_map
     - last_starting_action        (w) [starting.StartingAction]: last starting action gets set to unparked
    """
    def __init__(self, name="Save Parking Pose"):
        super(SaveParkingPoseFromOdom, self).__init__(name)
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

        self.blackboard.last_starting_action = starting.StartingAction.UNPARKED
        return py_trees.Status.SUCCESS


# class SaveLocation(py_trees.Behaviour):
#
#     last_parking = None
#
#     def __init__(self, name, homebase_blackboard_attr, publish_homebase_pose=False):
#         """:param homebase_blackboard_attr: the blackboard attribute which contains the
#             homebase location, in the same reference frame as the start location
#         :param publish_homebase_pose: if true, publish the homebase to the initial pose topic
#
#         """
#         super(SaveLocation, self).__init__(name)
#         self.gopher = gopher_configuration.Configuration()
#         self._location_publisher = rospy.Publisher(self.gopher.topics.initial_pose, geometry_msgs.PoseWithCovarianceStamped, queue_size=1)
#         self.blackboard = py_trees.Blackboard()
#         self.blackboard.last_parking = None
#         self.homebase_blackboard_attr = homebase_blackboard_attr
#         self.publish_homebase_pose = publish_homebase_pose
#
#     def update(self):
#         # if dslam is initialised, we have the map-relative position of
#         # the parking spot. Need to compute the relative position of the
#         # parking spot to the homebase - this is the difference
#         # transform between the parking spot and the homebase. We also
#         # check how much the parking spots differ. If the difference is
#         # small, we retain the parking spot that was used before to
#         # prevent drift in the parking locations.
#         new_parking = self.update_parking_location(getattr(self.blackboard, self.homebase_blackboard_attr))
#         # class variable so that different instances can use the same location
#         SaveLocation.last_parking = new_parking
#         self.blackboard.T_hb_to_parking = new_parking
#         self.blackboard.unpark_success = True
#
#         if self.publish_homebase_pose:
#             pose = geometry_msgs.PoseWithCovarianceStamped()
#             pose.header.stamp = rospy.Time.now()
#             pose.header.frame_id = "map"
#             pose.pose.pose = tf_utilities.transform_to_pose([self.blackboard.pose_homebase_rel_map.position,
#                                                              self.blackboard.pose_homebase_rel_map.orientation])
#             self._location_publisher.publish(pose)
#
#         self.blackboard.last_action_unparked = True
#         return py_trees.Status.SUCCESS
#
#     def update_parking_location(self, homebase_transform):
#         """Get the new parking location. The transform received is expected to be the
#         homebase location in the same frame as the start transform. When no
#         parking behaviour has been done before, both transforms are computed
#         from odometry. When dslam is initialised (usually after a single
#         delivery run), both use information from dslam instead. If the new
#         parking location is different enough from the previous one, the location
#         will be changed. This should help prevent the parking location drifting
#         due to slight errors in localisation.
#
#         """
#         if self.blackboard.last_parking is not None:
#             rospy.logdebug("last parking was nonempty - comparing the new transform to see if the parking location moved")
#             rospy.logdebug("last parking: " + tf_utilities.human_transform(self.blackboard.last_parking, oneline=True))
#             new_parking = tf_utilities.inverse_times(homebase_transform, self.blackboard.parking_start_transform)
#             rospy.logdebug("new parking: " + tf_utilities.human_transform(new_parking, oneline=True))
#             parking_diff = tf_utilities.inverse_times(self.blackboard.last_parking, new_parking)
#             rospy.logdebug("parking diff: " + tf_utilities.human_transform(parking_diff, oneline=True))
#
#             euc_diff = numpy.linalg.norm(parking_diff[0][:2])
#             rospy.logdebug("diff distance: " + str(euc_diff))
#             # extract yaw difference
#             yaw_diff = numpy.degrees(tf.transformations.euler_from_quaternion(parking_diff[1])[2])
#             rospy.logdebug("diff yaw: " + str(yaw_diff))
#
#             # if below threshold, keep the last parking location
#             if euc_diff < 1 and numpy.absolute(yaw_diff) < 90:
#                 rospy.logdebug("differences below threshold. keeping last parking location")
#                 new_parking = self.blackboard.last_parking
#             else:
#                 rospy.loginfo("UnPark : Difference from last parking location was above threshold. Parking location modified.")
#         else:
#             new_parking = tf_utilities.inverse_times(homebase_transform, self.blackboard.parking_start_transform)
#             rospy.logdebug("no previous parking location. Initialising to " + tf_utilities.human_transform(new_parking, oneline=True))
#
#         rospy.logdebug("new parking is " + str(new_parking))
#         return new_parking


# if __name__ == '__main__':
#     unpark = Unpark("unpark")
#
#     dslam_pub = rospy.Publisher("/dslam/diagnostics", dslam_msgs.Diagnostics, queue_size=1, latch=True)
#     if rospy.get_param('~dslam'):  # set dslam parameter to true to run the dslam branch
#         dslam_pub.publish(dslam_msgs.Diagnostics())
#
#     blackboard = py_trees.Blackboard()
#
#     tree = py_trees.ROSBehaviourTree(unpark)
#     tree.tick_tock(500)

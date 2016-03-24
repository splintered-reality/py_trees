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

----

"""

##############################################################################
# Imports
##############################################################################

import gopher_std_msgs.msg as gopher_std_msgs
import navigation
import py_trees

from . import starting
from . import transforms
from . import transform_utilities

##############################################################################
# Behaviours
##############################################################################


class Park(py_trees.Sequence):
    """
    Blackboard Variables:

     - pose_park_rel_map       (r) [geometry_msgs/Pose]                     : pose of the parking location relative to the homebase
     - last_starting_action    (r) [starting.StartingAction]                : checks this variable to see if parking is a valid move
     - pose_park_start_rel_map (w) [geometry_msgs.PoseWithCovarianceStamped]: transferred from /navi/pose when about to park
    """
    def __init__(self, name="Park"):
        super(Park, self).__init__(name)

        check_previously_parked = py_trees.CheckBlackboardVariable(
            name='Previously Unparked?',
            variable_name='last_starting_action',
            expected_value=starting.StartingAction.UNPARKED
        )
        write_pose_park_start_rel_map = navigation.create_map_pose_to_blackboard_behaviour(
            name="Start Pose (Map)",
            blackboard_variables={"pose_park_start_rel_map": None}
        )
        parking_motions = ParkingMotions()

        self.add_child(check_previously_parked)
        self.add_child(write_pose_park_start_rel_map)
        self.add_child(parking_motions)

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

    return (distance_to_park, point_to_park_angle, orient_with_park_angle)


class ParkingMotions(py_trees.Sequence):
    """
    Dynamically stitches together some simple motions on the fly (on entry via
    the initialise method with data from the blackboard) and adds them as children.
    These children are then dumped when the sequence is stopped or invalidated.

    Blackboard Variables:

     - pose_park_rel_map       (r) [geometry_msgs/Pose]                     : pose of the parking location relative to the homebase
     - pose_park_start_rel_map (w) [geometry_msgs.PoseWithCovarianceStamped]: transferred from /navi/pose when about to park
    """

    def __init__(self, name="ParkingMotions"):
        super(ParkingMotions, self).__init__(name)

        self.close_to_park_distance_threshold = 0.1
        self.blackboard = py_trees.Blackboard()

        # dummy children for dot graph rendering purposes
        # they will get cleaned up the first time this enters initialise
        point_to_park = navigation.SimpleMotion(
            name="Point to Park (?rad)",
            motion_type=gopher_std_msgs.SimpleMotionGoal.MOTION_ROTATE,
            motion_amount=0.0,
        )
        move_to_park = navigation.SimpleMotion(
            name="Move to Park (?m)",
            motion_type=gopher_std_msgs.SimpleMotionGoal.MOTION_TRANSLATE,
            motion_amount=0.0,
        )
        orient_with_park = navigation.SimpleMotion(
            name="Orient to Park (?rad)",
            motion_type=gopher_std_msgs.SimpleMotionGoal.MOTION_ROTATE,
            motion_amount=0.0,
        )
        self.add_child(point_to_park)
        self.add_child(move_to_park)
        self.add_child(orient_with_park)

    def initialise(self):
        super(ParkingMotions, self).initialise()

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
            point_to_park = navigation.SimpleMotion(
                name="Point to Park %0.2f rad" % point_to_park_angle,
                motion_type=gopher_std_msgs.SimpleMotionGoal.MOTION_ROTATE,
                motion_amount=point_to_park_angle,
            )
            self.add_child(point_to_park)
        if abs(distance_to_park) > epsilon:
            move_to_park = navigation.SimpleMotion(
                name="Move to Park %0.2f m" % distance_to_park,
                motion_type=gopher_std_msgs.SimpleMotionGoal.MOTION_TRANSLATE,
                motion_amount=distance_to_park,
            )
            self.add_child(move_to_park)
        if abs(orient_with_park_angle) > epsilon:
            orient_with_park = navigation.SimpleMotion(
                name="Orient to Park %0.2f rad" % orient_with_park_angle,
                motion_type=gopher_std_msgs.SimpleMotionGoal.MOTION_ROTATE,
                motion_amount=orient_with_park_angle,
            )
            self.add_child(orient_with_park)

    def stop(self, new_status=py_trees.Status.INVALID):
        super(ParkingMotions, self).stop(new_status)
#         # if deprioritised (INVALID), or newly initialised (RUNNING)
#         if new_status == py_trees.Status.INVALID or new_status == py_trees.Status.RUNNING:

# class InitParkingTransforms(py_trees.Behaviour):
#
#     def __init__(self, name):
#         super(InitParkingTransforms, self).__init__(name)
#         self.semantic_locations = Semantics(gopher_configuration.Configuration().namespaces.semantics)
#         self.blackboard = py_trees.Blackboard()
#         self.tf_listener = tf.TransformListener()
#
#     def initialise(self):
#         self.timeout = rospy.Time.now() + rospy.Duration(10)
#
#     def update(self):
#         # get the relative rotation and translation between the homebase and the current position
#         T_hb_to_current = tf_utilities.inverse_times(self.blackboard.homebase_transform, self.blackboard.delivery_end_tf)
#         # get the relative rotation between the homebase and the parking location
#         T_hb_to_parking = self.blackboard.T_hb_to_parking
#
#         # create transformstamped objects for the transformations from homebase to the two locations
#         t = tf.Transformer(True, rospy.Duration(10))
#         cur = tf_utilities.transform_to_transform_stamped(T_hb_to_current)
#         cur.header.frame_id = "homebase"
#         cur.child_frame_id = "current"
#         t.setTransform(cur)
#
#         park = tf_utilities.transform_to_transform_stamped(T_hb_to_parking)
#         park.header.frame_id = "homebase"
#         park.child_frame_id = "parking"
#         t.setTransform(park)
#
#         # Transform from current to parking computed by looking up the transform
#         # through the transformer. Current is our current frame of reference, so
#         # we know the location of the parking point in that frame. We need to
#         # compute the bearing to that point so that we can rotate to it, before
#         # moving in its direction.
#         T_current_to_parking = t.lookupTransform("current", "parking", rospy.Time(0))
#
#         # straight line distance between current location and parking
#         self.blackboard.parking_distance = norm(T_current_to_parking[0][:2])
#
#         # rotation to perform at the current position to align us with the
#         # parking location position so we can drive towards it
#         self.blackboard.rotation_to_parking = tf_utilities.transform_bearing(T_current_to_parking)
#
#         self.blackboard.rotation_at_parking = tf.transformations.euler_from_quaternion(T_current_to_parking[1])[2]
#
#         return py_trees.Status.SUCCESS
#
#
# class ParkingMotions2(py_trees.Behaviour):
#
#     def __init__(self, name):
#         super(ParkingMotions, self).__init__(name)
#         self.motion = navigation.SimpleMotion()
#         self.blackboard = py_trees.Blackboard()
#         rospy.on_shutdown(functools.partial(self.stop, py_trees.Status.FAILURE))
#
#     def initialise(self):
#         # if the distance to the parking spot is small, do not rotate and
#         # translate, just do the final rotation to face the same orientation.
#         if self.blackboard.parking_distance < 0.5:
#             self.motions_to_execute = [["rotate", self.blackboard.rotation_at_parking]]
#         else:
#             # rotation to perform when at the parking location to align us as
#             # before - need to subtract the rotation we made to face the parking
#             # location
#             self.blackboard.rotation_at_parking = self.blackboard.rotation_at_parking - self.blackboard.rotation_to_parking
#             self.motions_to_execute = [["rotate", self.blackboard.rotation_to_parking],
#                                        ["translate", self.blackboard.parking_distance], ["rotate", self.blackboard.rotation_at_parking]]
#
#         # Start execution of the rotation motion which will orient us facing the
#         # parking location. Update will check if the motion is complete and
#         # whether or not it was successful
#         rospy.loginfo("Parking : executing motions {0}".format(self.motions_to_execute))
#         next_motion = self.motions_to_execute.pop(0)
#         self.motion.execute(next_motion[0], next_motion[1])
#
#     def update(self):
#         # if the motion isn't complete, we're running
#         if not self.motion.complete():
#             rospy.logdebug("Park : waiting for motion to complete")
#             self.feedback_message = "waiting for motion to complete"
#             return py_trees.Status.RUNNING
#         else:
#             # Otherwise, the motion is finished. If either of the motions fail,
#             # the behaviour fails.
#             if not self.motion.success():
#                 self.blackboard.unpark_success = False
#                 rospy.logdebug("Park : motion failed")
#                 self.feedback_message = "motion failed"
# #                self.log_parking_location(False)
#                 return py_trees.Status.FAILURE
#             else:
#                 # The motion successfully completed. Check if there is another
#                 # motion to do, and execute it if there is, otherwise return
#                 # success
#                 if len(self.motions_to_execute) == 0:
#                     rospy.logdebug("Park : successfully completed all motions")
#                     self.feedback_message = "successfully completed all motions"
#  #                   self.log_parking_location(True)
#                     return py_trees.Status.SUCCESS
#                 else:
#                     next_motion = self.motions_to_execute.pop(0)
#                     self.motion.execute(next_motion[0], next_motion[1])
#                     rospy.logdebug("Park : waiting for motion to complete")
#                     self.feedback_message = "waiting for motion to complete"
#                     return py_trees.Status.RUNNING

    # def log_parking_location(self, success):
    #     """Debug function to store some information about parking locations used.
    #     """
    #     self.tf_listener.waitForTransform("map", "base_link", rospy.Time(), rospy.Duration(1))
    #     # do not replace self.current_transform
    #     current_transform = self.tf_listener.lookupTransform("map", "base_link", rospy.Time(0))
    #     t = tf.Transformer(True, rospy.Duration(10))
    #     cur = tf_utilities.transform_to_transform_stamped(current_transform)
    #     cur.header.frame_id = "map"
    #     cur.child_frame_id = "current"
    #     t.setTransform(cur)

    #     hb = tf_utilities.transform_to_transform_stamped(self.hb)
    #     hb.header.frame_id = "map"
    #     hb.child_frame_id = "homebase"
    #     t.setTransform(hb)

    #     park = tf_utilities.transform_to_transform_stamped(self.blackboard.T_hb_to_parking)
    #     park.header.frame_id = "homebase"
    #     park.child_frame_id = "parking"
    #     t.setTransform(park)

    #     T_current_to_parking = t.lookupTransform("current", "parking", rospy.Time(0))
    #     T_map_to_parking = t.lookupTransform("map", "parking", rospy.Time(0))
    #     with open("/opt/groot/parkinglog-" + str(Park.time) + ".txt", "a") as f:
    #         if success:
    #             f.write("--------------------SUCCESS--------------------\n")
    #         else:
    #             f.write("--------------------FAILURE--------------------\n")
    #         f.write("homebase" + tf_utilities.human_transform(self.hb, oneline=True) + "\n")
    #         f.write("parking" + tf_utilities.human_transform(T_map_to_parking, oneline=True) + "\n")
    #         f.write("finishing" + tf_utilities.human_transform(current_transform, oneline=True) + "\n")
    #         f.write("disparity" + tf_utilities.human_transform(T_current_to_parking, oneline=True) + "\n")

#     def stop(self, new_status):
#         self.motion.stop()
#
# if __name__ == '__main__':
#     # assumes simulation is running
#     rospy.init_node("park_test")
#
#     blackboard = py_trees.blackboard.Blackboard()
#     blackboard.__shared_state = {}
#
#     park = Park("park")
#
#     dslam_pub = rospy.Publisher("/dslam/diagnostics", dslam_msgs.Diagnostics, queue_size=1, latch=True)
#
#     # initialise a transform with the pose of the station
#     ds_translation = (1, 1, 0)
#     # quaternion specified by rotating theta radians around the yaw axis
#     ds_rotation = tuple(tf.transformations.quaternion_about_axis(0.2, (0, 0, 1)))
#     hb_translation = (0, 0, 0)
#     hb_rotation = tuple(tf.transformations.quaternion_about_axis(0, (0, 0, 1)))
#
#     blackboard.T_hb_to_parking = tf_utilities.inverse_times((hb_translation, hb_rotation), (ds_translation, ds_rotation))
#
#     tree = py_trees.ROSBehaviourTree(park)
#     tree.tick_tock(500)

#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/yujinrobot/gopher_crazy_hospital/license/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
.. module:: moveit
   :platform: Unix
   :synopsis: Moving behaviours

Kick the gophers around with these behaviours.

----

"""

##############################################################################
# Imports
##############################################################################

import actionlib
from actionlib_msgs.msg import GoalStatus
import move_base_msgs.msg as move_base_msgs
import gopher_std_msgs.msg as gopher_std_msgs
import std_msgs.msg as std_msgs
import dslam_msgs.msg as dslam_msgs
import somanet_msgs.msg as somanet_msgs
import geometry_msgs.msg as geometry_msgs
import ar_track_alvar_msgs.msg as ar_track_alvar_msgs
import py_trees
import tf
import rospy
import numpy
from .time import Pause
from gopher_semantics.semantics import Semantics
from gopher_semantics.docking_stations import DockingStations
from .blackboard import Blackboard

##############################################################################
# Functions
##############################################################################


# compute tf1 multiplied by the inverse of tf2 - this gives the transformation
# between the two transformations. Assumes transforms in the form ((x,y,z),
# (x,y,z,w)) where the first part of the tuple is the translation and the second
# part the rotation quaternion. The result is the transform T_tf2_rel_tf1 or
# T^{tf2}_{tf1} or T_tf1_to_tf2

# Based on code at
# http://docs.ros.org/jade/api/tf/html/c++/Transform_8h_source.html
def inverse_times(tf1, tf2):
    # matrix to allow vector-matrix multiplication to generate a vector
    # this is the difference between the translations of the two transforms
    v = numpy.matrix(tf2[0]) - numpy.matrix(tf1[0])
    # create quaternion matrices from the vectors
    q1 = numpy.matrix(tf.transformations.quaternion_matrix(tf1[1]))
    q2 = numpy.matrix(tf.transformations.quaternion_matrix(tf2[1]))
    # difference between the quaternions of the two transforms - relative
    # rotation
    diffq = tf.transformations.quaternion_from_matrix(numpy.transpose(q1) * q2)
    # strip row+column 4 from q1 to multiply with vector
    q1nonhom = q1[numpy.ix_([0, 1, 2], [0, 1, 2])]
    difft = v * q1nonhom

    return (tuple(numpy.array(difft)[0]), tuple(diffq))


# convert a transform to a human readable string
def human_transform(t):
    return "x translation: " + str(t[0][0]) + "\ny translation: " + str(t[0][1]) + "\n yaw: " + str(numpy.degrees(tf.transformations.euler_from_quaternion(t[1])[2]))


def transform_to_pose(t):
    pose = geometry_msgs.Pose()
    pose.position.x = t[0][0]
    pose.position.y = t[0][1]
    pose.position.z = t[0][2]
    pose.orientation.x = t[1][0]
    pose.orientation.y = t[1][1]
    pose.orientation.z = t[1][2]
    pose.orientation.w = t[1][3]

    return pose


def transform_to_transform_stamped(t):
    stamped = geometry_msgs.TransformStamped()

    stamped.transform.translation.x = t[0][0]
    stamped.transform.translation.y = t[0][1]
    stamped.transform.translation.z = t[0][2]

    stamped.transform.rotation.x = t[1][0]
    stamped.transform.rotation.y = t[1][1]
    stamped.transform.rotation.z = t[1][2]
    stamped.transform.rotation.w = t[1][3]

    return stamped


# get the bearing of a transform in the coordinate frame of the transform which
# the given transform is relative to, i.e. given T_homebase_to_dock, this
# function will give you the bearing of the dock from the homebase, in the coordinate frame of the homebase
def transform_bearing(transform):
    # compute the angle between the x axis and the translation vector of the other transform
    xaxis = [1, 0]
    # the translation is the position of the other point of interest in the
    # parent frame
    translation = [transform[0][0], transform[0][1]]
    cross = numpy.cross(xaxis, translation)  # the sign tells us whether this is a positive or negative rotation
    # tan formula from https://newtonexcelbach.wordpress.com/2014/03/01/the-angle-between-two-vectors-python-version/
    return numpy.sign(cross) * numpy.arctan2(numpy.linalg.norm(cross), numpy.dot(xaxis, translation))

##############################################################################
# Combined behaviour classes
##############################################################################


class Starting(py_trees.Selector):
    def __init__(self, name):
        undockseq = py_trees.Sequence("Undock", [IsDocked("Check for dock"), Undock("Undock"), Pause("Pause", 2)])
        unparkseq = py_trees.Sequence("Unpark", [Unpark("Unpark"), Pause("Pause", 2)])
        children = [undockseq, unparkseq]
        super(Starting, self).__init__(name, children)


class Finishing(py_trees.Selector):
    def __init__(self, name):
        dodock = py_trees.Sequence("Dock/notify", [Dock("Dock"), NotifyComplete("Notify")])
        dockseq = py_trees.Sequence("Maybe dock", [WasDocked("Was I docked?"), dodock])

        dopark = py_trees.Sequence("Park/notify", [Park("Park"), NotifyComplete("Notify")])
        parkseq = py_trees.Sequence("Maybe park", [py_trees.meta.inverter(WasDocked("Was I parked?")), dopark])

        children = [parkseq, dockseq, WaitForCharge("Wait for help")]
        super(Finishing, self).__init__(name, children)

##############################################################################
# Class
##############################################################################


class SimpleMotion():
    def __init__(self):
        self.action_client = actionlib.SimpleActionClient('~simple_motion_controller', gopher_std_msgs.SimpleMotionAction)
        self.has_goal = False

    # Motion is either rotation or translation, value is corresponding rotation in radians, or distance in metres
    def execute(self, motion, value):
        self.connected = self.action_client.wait_for_server(rospy.Duration(0.5))
        if not self.connected:
            rospy.logwarn("SimpleMotion : could not connect to simple motion controller server.")
            return False

        goal = gopher_std_msgs.SimpleMotionGoal()
        if motion == "rotate":
            goal.motion_type = gopher_std_msgs.SimpleMotionGoal.MOTION_ROTATE
        elif motion == "translate":
            goal.motion_type = gopher_std_msgs.SimpleMotionGoal.MOTION_TRANSLATE
        else:
            rospy.logerr("SimpleMotion : received unknown motion type {0}".format(motion))
            return False

        self.has_goal = True
        goal.motion_amount = value
        goal.unsafe = False
        self.action_client.send_goal(goal)

    def complete(self):
        result = self.action_client.get_result()
        if not result:
            return False

        self.has_goal = False
        return True

    def get_status(self):
        return self.action_client.get_state()

    def success(self):
        result = self.action_client.get_result()
        if not result:
            return False
        elif result.value == gopher_std_msgs.SimpleMotionResult.SUCCESS:
            self.has_goal = False
            return True
        else:  # anything else is a failure state
            return False

    def abort(self):
        if self.has_goal:
            motion_state = self.action_client.get_state()
            if (motion_state == GoalStatus.PENDING or motion_state == GoalStatus.ACTIVE):
                self.action_client.cancel_goal()


class IsDocked(py_trees.Behaviour):
    def __init__(self, name):
        super(IsDocked, self).__init__(name)
        self._battery_subscriber = self._battery_subscriber = rospy.Subscriber("~battery", somanet_msgs.SmartBatteryStatus, self.battery_callback)
        self.docked = None

    def battery_callback(self, msg):
        self.docked = msg.charging_source == somanet_msgs.SmartBatteryStatus.CHARGING_SOURCE_DOCK

    def update(self):
        if self.docked is None:  # no message yet
            return py_trees.Status.RUNNING
        elif self.docked:  # is docked
            return py_trees.Status.SUCCESS
        else:  # is not docked
            return py_trees.Status.FAILURE


class WasDocked(py_trees.Behaviour):
    def __init__(self, name):
        super(WasDocked, self).__init__(name)
        self.blackboard = Blackboard()

    def update(self):
        if not hasattr(self.blackboard, 'parked'):
            return py_trees.Status.FAILURE
        elif self.blackboard.parked:
            return py_trees.Status.FAILURE
        else:
            return py_trees.Status.SUCCESS


class Park(py_trees.Behaviour):
    def __init__(self, name):
        super(Park, self).__init__(name)
        self.tf_listener = tf.TransformListener()
        self.semantic_locations = Semantics()
        self.motion = SimpleMotion()
        self.blackboard = Blackboard()

    def initialise(self):
        self.not_unparked = False
        self.start_transform = None

        # if the homebase to dock transform is unset, we didn't do an unparking
        # action - this could mean that the run was started without doing any
        # undock or unpark action
        if self.blackboard.T_hb_to_parking is None:
            self.not_unparked = True
            return

        self.homebase = self.semantic_locations.semantic_modules['locations']['homebase']
        # assume map frame is 0,0,0 with no rotation; translation of
        # homebase is the position of the homebase specified in semantic
        # locations
        hb_translation = (self.homebase.pose.x, self.homebase.pose.y, 0)
        # quaternion specified by rotating theta radians around the yaw axis
        hb_rotation = tuple(tf.transformations.quaternion_about_axis(self.homebase.pose.theta, (0, 0, 1)))

        rospy.logdebug("Park : waiting for transform from map to base_link")
        # get the current position of the robot as a transform from map to base link
        try:
            self.tf_listener.waitForTransform("map", "base_link", rospy.Time(), rospy.Duration(1))
            self.start_transform = self.tf_listener.lookupTransform("map", "base_link", rospy.Time(0))
        except tf.Exception:
            rospy.logerr("Park : failed to get transform from map to base_link")
            return

        # get the relative rotation and translation between the homebase and the current position
        T_hb_to_current = inverse_times((hb_translation, hb_rotation), self.start_transform)
        # get the relative rotation between the homebase and the parking location
        T_hb_to_parking = self.blackboard.T_hb_to_parking

        # create transformstamped objects for the transformations from homebase to the two locations
        t = tf.Transformer(True, rospy.Duration(10))
        cur = transform_to_transform_stamped(T_hb_to_current)
        cur.header.frame_id = "homebase"
        cur.child_frame_id = "current"
        t.setTransform(cur)

        park = transform_to_transform_stamped(T_hb_to_parking)
        park.header.frame_id = "homebase"
        park.child_frame_id = "parking"
        t.setTransform(park)

        # Transform from current to parking computed by looking up the transform
        # through the transformer. Current is our current frame of reference, so
        # we know the location of the parking point in that frame. We need to
        # compute the bearing to that point so that we can rotate to it, before
        # moving in its direction.
        T_current_to_parking = t.lookupTransform("current", "parking", rospy.Time(0))

        # straight line distance between current location and parking
        self.distance = numpy.linalg.norm(T_current_to_parking[0][:2])

        # rotation to perform at the current position to align us with the
        # parking location position so we can drive towards it
        self.rotation_to_parking = transform_bearing(T_current_to_parking)

        # if the distance to the parking spot is small, do not rotate and
        # translate, just do the final translation to face the same orientation.
        if self.distance < 0.5:
            self.rotation_at_parking = tf.transformations.euler_from_quaternion(T_current_to_parking[1])[2]
            self.motions_to_execute = [["rotate", self.rotation_at_parking]]
        else:
            # rotation to perform when at the parking location to align us as
            # before - need to subtract the rotation we made to face the parking
            # location
            self.rotation_at_parking = tf.transformations.euler_from_quaternion(T_current_to_parking[1])[2] - self.rotation_to_parking
            self.motions_to_execute = [["rotate", self.rotation_to_parking],
                                       ["translate", self.distance], ["rotate", self.rotation_at_parking]]

        # Start execution of the rotation motion which will orient us facing the
        # parking location. Update will check if the motion is complete and
        # whether or not it was successful
        motion = self.motions_to_execute.pop(0)
        self.motion.execute(motion[0], motion[1])

    def update(self):
        if self.start_transform is None:
            rospy.logdebug("Park : could not get current transform from map to base_link")
            self.feedback_message = "could not get current transform from map to base_link"
            return py_trees.Status.FAILURE
        if self.not_unparked:
            rospy.logdebug("Park : transform to parking is undefined - probably no unpark action was performed")
            self.feedback_message = "transform to parking is undefined - probably no unpark action was performed"
            return py_trees.Status.FAILURE
        # if the motion isn't complete, we're running
        if not self.motion.complete():
            rospy.logdebug("Park : waiting for motion to complete")
            self.feedback_message = "waiting for motion to complete"
            return py_trees.Status.RUNNING
        else:
            # Otherwise, the motion is finished. If either of the motions fail,
            # the behaviour fails.
            if not self.motion.success():
                self.blackboard.unpark_success = False
                rospy.logdebug("Park : motion failed")
                self.feedback_message = "motion failed"
                return py_trees.Status.FAILURE
            else:
                # The motion successfully completed. Check if there is another
                # motion to do, and execute it if there is, otherwise return
                # success
                if len(self.motions_to_execute) == 0:
                    rospy.logdebug("Park : successfully completed all motions")
                    self.feedback_message = "successfully completed all motions"
                    return py_trees.Status.SUCCESS
                else:
                    motion = self.motions_to_execute.pop(0)
                    self.motion.execute(motion[0], motion[1])
                    rospy.logdebug("Park : waiting for motion to complete")
                    self.feedback_message = "waiting for motion to complete"
                    return py_trees.Status.RUNNING

    def abort(self, new_status):
        self.motion.abort()
        # set to none to use later as a flag to indicate not having performed
        # unparking behaviour
        if new_status == py_trees.Status.FAILURE:
            self.blackboard.T_hb_to_parking = None


class Unpark(py_trees.Behaviour):
    def __init__(self, name):
        super(Unpark, self).__init__(name)
        self._notify_publisher = rospy.Publisher("~display_notification", gopher_std_msgs.Notification, queue_size=1)
        self._location_publisher = rospy.Publisher("/navi/initial_pose", geometry_msgs.PoseWithCovarianceStamped, queue_size=1)

        self.tf_listener = tf.TransformListener()
        self.blackboard = Blackboard()
        # use unpark success variable to allow dslam-based unparking only when
        # dslam actually has an accurate location lock. If we never successfully
        # unparked before, this is unlikely to be the case.
        if not hasattr(self.blackboard, 'unpark_success'):
            self.blackboard.unpark_success = False

        if not hasattr(self.blackboard, 'unpark_first_run'):
            self.blackboard.unpark_first_run = True
        self.semantic_locations = Semantics()

    def initialise(self):
        self.last_dslam = None
        self.button_pressed = False
        # indicate whether transforms have been setup from map or odom to base link
        self.transform_setup = False
        self.homebase = None  # set this in the initialise() step
        self.end_transform = None
        self.start_transform = None
        self.lookup_attempts = 0
        self.blackboard.parked = True  # set this to true to indicate that the robot was parked, for the wasdocked behaviour

        # only initialise subscribers when the behaviour starts running
        self.homebase = self.semantic_locations.semantic_modules['locations']['homebase']
        self._battery_subscriber = self._battery_subscriber = rospy.Subscriber("~battery", somanet_msgs.SmartBatteryStatus, self.battery_callback)
        self._button_subscriber = rospy.Subscriber("/gopher/buttons/go", std_msgs.Empty, self.button_callback)
        self._dslam_subscriber = rospy.Subscriber("/dslam/diagnostics", dslam_msgs.Diagnostics, self.dslam_callback)

        # assume map frame is 0,0,0 with no rotation; translation of homebase is
        # the position of the homebase specified in semantic locations. We need
        # the homebase location regardless of whether dslam is initialised.
        self.hb_translation = (self.homebase.pose.x, self.homebase.pose.y, 0)
        # quaternion specified by rotating theta radians around the yaw axis
        self.hb_rotation = tuple(tf.transformations.quaternion_about_axis(self.homebase.pose.theta, (0, 0, 1)))

        # Wait on a message from dslam for a bit. If one doesn't show up, assume dslam isn't initialised
        self.timeout = rospy.Time.now() + rospy.Duration(3)

    def battery_callback(self, msg):
        self.still_charging = msg.charge_state == somanet_msgs.SmartBatteryStatus.CHARGING

    def button_callback(self, msg):
        self.button_pressed = True
        self._notify_publisher.publish(gopher_std_msgs.Notification(led_pattern=gopher_std_msgs.Notification.CANCEL_CURRENT,
                                                                    button_confirm=gopher_std_msgs.Notification.BUTTON_OFF,
                                                                    button_cancel=gopher_std_msgs.Notification.RETAIN_PREVIOUS,
                                                                    message="button was pressed"))
        # when the button is pressed, save the latest odom value so we can get
        # the transform between the start and end poses
        try:
            self.end_transform = self.tf_listener.lookupTransform("odom", "base_link", rospy.Time(0))
        except tf.Exception:
            rospy.logerr("Unpark : failed to lookup transform from odom to base link on button press")

    def dslam_callback(self, msg):
        self.last_dslam = msg

    def update(self):
        if not self.transform_setup:
            if not self.last_dslam:  # no message from dslam yet
                # waited longer than the timeout, so assume dslam is uninitialised and continue
                if rospy.Time.now() > self.timeout or self.blackboard.unpark_first_run:
                    rospy.logdebug("Unpark : didn't get a message from dslam - assuming uninitialised")
                    self.feedback_message = "no message from dslam - assuming uninitialised"
                    self.dslam_initialised = False
                    self.blackboard.unpark_first_run = False
                else:
                    rospy.logdebug("Unpark : waiting for dslam state update")
                    self.feedback_message = "waiting for dslam state update"
                    return py_trees.Status.RUNNING
            else:
                rospy.logdebug("Unpark : got dslam message")
                self.dslam_initialised = self.last_dslam.state == dslam_msgs.Diagnostics.WORKING

            if self.dslam_initialised and self.blackboard.unpark_success:
                # if dslam is initialised, the map->base_link transform gives us the
                # actual position of the robot, i.e. the position of the parking
                # spot. Save this as the starting position
                rospy.logdebug("Unpark : waiting for transform from map to base_link")
                try:
                    self.start_transform = self.tf_listener.lookupTransform("map", "base_link", rospy.Time(0))
                    self.transform_setup = True
                except tf.Exception:
                    self.lookup_attempts += 1
                    self.feedback_message = "waiting for parking location transform from dslam"
                    return py_trees.Status.RUNNING
            else:
                # save the latest odom message received so that we can use it to compute
                # a transform back to the starting position
                rospy.logdebug("Unpark : waiting for transform from odom to base_link")
                try:
                    self.start_transform = self.tf_listener.lookupTransform("odom", "base_link", rospy.Time(0))
                    self.transform_setup = True
                except tf.Exception:
                    self.lookup_attempts += 1
                    self.feedback_message = "waiting for parking location odometry transform"
                    return py_trees.Status.RUNNING

            if self.lookup_attempts >= 4:
                rospy.logerr("Unpark : failed to get transform for current location")
                self.feedback_message = "failed to get transform for current location"
                return py_trees.Status.FAILURE

        # need to be unplugged before we can move or be moved
        if self.still_charging:
            rospy.logdebug("Unpark : robot is still charging - waiting to be unplugged")
            self._notify_publisher.publish(gopher_std_msgs.Notification(led_pattern=gopher_std_msgs.Notification.FLASH_YELLOW,
                                                                        button_cancel=gopher_std_msgs.Notification.RETAIN_PREVIOUS,
                                                                        button_confirm=gopher_std_msgs.Notification.RETAIN_PREVIOUS,
                                                                        message="waiting to be unplugged"))
            # reset the button just in case it was accidentally pressed
            self.feedback_message = "robot is still charging - waiting to be unplugged"
            self.button_pressed = False
        else:  # was unplugged, should start moving
            if self.dslam_initialised and self.blackboard.unpark_success:
                # if dslam is initialised, we have the map-relative position of
                # the parking spot. Need to compute the relative position of the
                # parking spot to the homebase - this is the difference
                # transform between the parking spot and the homebase

                # transform homebase to parking spot
                self.blackboard.T_hb_to_parking = inverse_times((self.hb_translation, self.hb_rotation), self.start_transform)
                self.blackboard.unpark_success = True
                rospy.logdebug("Unpark : successfully unparked")
                self.feedback_message = "successfully unparked"
                return py_trees.Status.SUCCESS
            else:
                rospy.logdebug("Unpark : waiting for go button to be pressed to indicate homebase")
                # otherwise, the robot needs to be guided to the homebase by
                # someone - pressing the go button indicates that the homebase
                # has been reached.
                if self.button_pressed:
                    if self.end_transform is None:
                        rospy.logerr("Unpark : did not get a homebase transform")
                        self.feedback_message = "did not get a homebase transform"
                        return py_trees.Status.FAILURE

                    self.blackboard.T_hb_to_parking = inverse_times(self.end_transform, self.start_transform)

                    pose = geometry_msgs.PoseWithCovarianceStamped()
                    pose.header.stamp = rospy.Time.now()
                    pose.header.frame_id = "map"
                    pose.pose.pose = transform_to_pose((self.hb_translation, self.hb_rotation))
                    self._location_publisher.publish(pose)
                    self.blackboard.unpark_success = True
                    rospy.logdebug("Unpark : Successfully unparked")
                    self.feedback_message = "successfully unparked"
                    return py_trees.Status.SUCCESS
                else:
                    # publish notification request to flash and turn on the go button led
                    self._notify_publisher.publish(gopher_std_msgs.Notification(led_pattern=gopher_std_msgs.Notification.FLASH_PURPLE,
                                                                                button_confirm=gopher_std_msgs.Notification.BUTTON_ON,
                                                                                button_cancel=gopher_std_msgs.Notification.RETAIN_PREVIOUS,
                                                                                message="Unpark : waiting for go button press to continue"))
        return py_trees.Status.RUNNING


class Dock(py_trees.Behaviour):
    """
    Engage the docking behaviour
    """
    def __init__(self, name):
        super(Dock, self).__init__(name)
        self.blackboard = Blackboard()
        self.semantic_locations = Semantics()
        self.tf_listener = tf.TransformListener()
        self.motion = SimpleMotion()
        self.goal_sent = False
        self.docking_client = actionlib.SimpleActionClient('~autonomous_docking', gopher_std_msgs.AutonomousDockingAction)

        self._interrupt_sub = None
        try:
            if rospy.get_param("~enable_dock_interrupt"):
                self._interrupt_sub = rospy.Subscriber("/gopher/buttons/stop", std_msgs.Empty, self.interrupt_cb)
        except KeyError:
            pass

        self._honk_publisher = None
        try:
            if rospy.get_param("~enable_honks") and rospy.get_param("~docking_honk"):
                honk_topic = rospy.get_param("~docking_honk")
                self._honk_publisher = rospy.Publisher("/gopher/commands/sounds/" + honk_topic, std_msgs.Empty, queue_size=1)
        except KeyError:
            rospy.logwarn("Gopher Deliveries : Could not find param to initialise honks.")
            pass

    def initialise(self):

        if self.status == py_trees.Status.FAILURE:
            return

        self.interrupted = False  # button pressed to interrupt docking procedure?
        self.goal_sent = False  # docking goal sent?
        self.not_undocked = False
        self.got_transform = False
        self.timeout = rospy.Time.now() + rospy.Duration(3)

        # if the homebase to dock transform is unset, we didn't do a docking
        # action - this could mean that the run was started without doing any
        # undock or unpark action
        if self.blackboard.T_homebase_to_dock is None:
            self.not_undocked = True
            return

        self.homebase = self.semantic_locations.semantic_modules['locations']['homebase']
        self.hb_translation = (self.homebase.pose.x, self.homebase.pose.y, 0)
        self.hb_rotation = tuple(tf.transformations.quaternion_about_axis(self.homebase.pose.theta, (0, 0, 1)))

        if self._honk_publisher:
            self._honk_publisher.publish(std_msgs.Empty())

    def interrupt_cb(self, msg):
        rospy.logdebug("Dock behaviour got interrupt from button")
        self.interrupted = True
        self.docking_client.cancel_goal()
        self.motion.abort()

    def update(self):
        if self.interrupted:
            rospy.logdebug("Dock : interrupted")
            self.feedback_message = "docking was interrupted by button press"
            return py_trees.Status.FAILURE

        if self.not_undocked:
            rospy.logdebug("Dock : Transform to dock was unset - no undock action was performed")
            self.feedback_message = "Transform to dock was unset - no undock action was performed"
            return py_trees.Status.FAILURE

        if self.timeout > rospy.Time.now() and not self.got_transform:
            rospy.logdebug("Dock : waiting for transform from map to base_link")
            try:
                start_transform = self.tf_listener.lookupTransform("map", "base_link", rospy.Time(0))
            except tf.Exception:
                self.feedback_message = "waiting for transform from map to base_link"
                return py_trees.Status.RUNNING

            rospy.logdebug("Dock : got transform")
            T_hb_to_current = inverse_times((self.hb_translation, self.hb_rotation), start_transform)
            T_hb_to_dock = self.blackboard.T_homebase_to_dock

            rospy.logdebug("Dock : transform from homebase to current location")
            rospy.logdebug(human_transform(T_hb_to_current))
            rospy.logdebug("Dock : transform from homebase to docking location")
            rospy.logdebug(human_transform(T_hb_to_dock))

            # create transformstamped objects for the transformations from homebase to the two locations
            t = tf.Transformer(True, rospy.Duration(10))
            cur = transform_to_transform_stamped(T_hb_to_current)
            cur.header.frame_id = "homebase"
            cur.child_frame_id = "current"
            t.setTransform(cur)

            dock = transform_to_transform_stamped(T_hb_to_dock)
            dock.header.frame_id = "homebase"
            dock.child_frame_id = "docking"
            t.setTransform(dock)

            T_current_to_dock = t.lookupTransform("current", "docking", rospy.Time(0))

            rospy.logdebug("Dock : transform from current location to dock")
            rospy.logdebug(human_transform(T_current_to_dock))

            self.motion.execute("rotate", transform_bearing(T_current_to_dock))
            rospy.logdebug("Dock : rotating to face docking station")
            self.feedback_message = "rotating to face docking station"
            self.got_transform = True
            return py_trees.Status.RUNNING

        if not self.got_transform:
            rospy.logdebug("Dock : could not get transform from map to base link")
            self.feedback_message = "Could not get transform from map to base link"
            return py_trees.Status.FAILURE

        # running if the rotation motion is still executing
        if not self.motion.complete():
            rospy.logdebug("Dock : rotating to face docking station")
            self.feedback_message = "Rotating to face docking station"
            return py_trees.Status.RUNNING

        # if the rotation motion fails, the behaviour fails
        if not self.motion.success():
            rospy.logerr("Dock : failed to rotate to docking station")
            self.feedback_message = "Failed to rotate to docking station"
            return py_trees.Status.FAILURE
        else:
            if not self.goal_sent:
                self.connected = self.docking_client.wait_for_server(rospy.Duration(0.5))
                if not self.connected:
                    rospy.logerr("Dock : could not connect to autonomous docking server.")
                    self.feedback_message = "Action client failed to connect"
                    return py_trees.Status.INVALID

                # the rotation succeeded, now send the docking goal
                goal = gopher_std_msgs.AutonomousDockingGoal()
                goal.command = gopher_std_msgs.AutonomousDockingGoal.DOCK
                self.docking_client.send_goal(goal)
                self.goal_sent = True

        # at this point, we have sent the docking goal - check whether it's
        # succeeded or failed, or if it's still running.
        result = self.docking_client.get_result()
        if result:
            if result.value == gopher_std_msgs.AutonomousDockingResult.SUCCESS:
                rospy.logdebug("Dock : " + result.message)
                self.feedback_message = result.message
                return py_trees.Status.SUCCESS
            elif result.value == gopher_std_msgs.AutonomousDockingResult.ABORTED_OBSTACLES:
                rospy.logdebug("Dock : " + result.message)
                self.feedback_message = result.message
                return py_trees.Status.FAILURE
            else:
                rospy.logdebug("Dock : got unknown result value from docking controller")
                self.feedback_message = "Got unknown result value from docking controller"
                return py_trees.Status.FAILURE
        else:
            rospy.logdebug("Dock : docking in progress")
            self.feedback_message = "Docking in progress"
            return py_trees.Status.RUNNING

    def abort(self, new_status):
        self.motion.abort()
        if self.goal_sent:
            self.docking_client.cancel_goal()
        # reset the homebase to dock transform to none so we can use it as a
        # flag to indicate that there was no undock performed
        if new_status == py_trees.Status.FAILURE:
            self.blackboard.T_homebase_to_dock = None


class Undock(py_trees.Behaviour):
    """
    Leave the docking bay. This is typically a specialised maneuvre so that
    the robot can begin localising.
    """
    def __init__(self, name):
        super(Undock, self).__init__(name)
        self.action_client = actionlib.SimpleActionClient("~autonomous_docking", gopher_std_msgs.AutonomousDockingAction)
        self._location_publisher = rospy.Publisher("/navi/initial_pose", geometry_msgs.PoseWithCovarianceStamped, queue_size=1)
        self._honk_publisher = None
        self.blackboard = Blackboard()
        self.docking_stations = DockingStations()
        self.semantic_locations = Semantics()
        self.sent_goal = False
        self.got_markers = False
        self.blackboard.T_homebase_to_dock = None

        try:
            if rospy.get_param("~enable_honks") and rospy.get_param("~undocking_honk"):
                honk_topic = rospy.get_param("~undocking_honk")
                self._honk_publisher = rospy.Publisher("/gopher/commands/sounds/" + honk_topic, std_msgs.Empty, queue_size=1)
        except KeyError:
            rospy.logwarn("Gopher Deliveries : Could not find param to initialise honks.")
            pass

    def initialise(self):
        self.homebase = self.semantic_locations.semantic_modules['locations']['homebase']
        self.blackboard.parked = False
        self.sent_goal = False
        self.got_markers = False
        self.ar_markers_short = None
        self._ar_short_subscriber = rospy.Subscriber("ar_pose_marker_short_range", ar_track_alvar_msgs.AlvarMarkers, self.ar_cb_short)

        # wait a while for ar messages to come in
        self.timeout = rospy.Time.now() + rospy.Duration(3)

    def ar_cb_short(self, msg):
        # can have multiple short range markers
        self.ar_markers_short = [marker.id for marker in msg.markers]
        if self.ar_markers_short:
            self.got_markers = True

    def update(self):
        self.logger.debug("  %s [Undock::update()]" % self.name)
        # need to wait for AR marker information to come through

        if not self.got_markers:
            if rospy.Time.now() > self.timeout:
                rospy.logerr("Undock : could not find AR marker information")
                self.feedback_message = "did not get any AR marker information"
                return py_trees.Status.FAILURE
            else:
                rospy.logdebug("Undock : waiting for AR marker information")
                self.feedback_message = "waiting for AR marker information"
                return py_trees.Status.RUNNING

        if not self.sent_goal:
            self.blackboard.dock_ar_markers_small = self.ar_markers_short

            # retrieve information about this docking station
            stations = self.docking_stations.find_docking_stations_with_ar_marker_id(id_list=self.ar_markers_short)
            ds = None
            for station in stations.values():
                if (station.left_id == self.ar_markers_short[0] or station.left_id == self.ar_markers_short[1])\
                   and (station.right_id == self.ar_markers_short[0] or station.right_id == self.ar_markers_short[1]):
                    ds = station
                    break

            if ds is None:
                rospy.logerr("Undock : could not find docking station in semantic locations with visible markers")
                self.feedback_message = "could not find docking station in semantic locations with the visible markers"
                return py_trees.Status.FAILURE


            # initialise a transform with the pose of the station
            ds_translation = (ds.pose.x, ds.pose.y, 0)
            # quaternion specified by rotating theta radians around the yaw axis
            ds_rotation = tuple(tf.transformations.quaternion_about_axis(ds.pose.theta, (0, 0, 1)))
            hb_translation = (self.homebase.pose.x, self.homebase.pose.y, 0)
            hb_rotation = tuple(tf.transformations.quaternion_about_axis(self.homebase.pose.theta, (0, 0, 1)))

            # publish that information to initialise navigation, and save for the
            # docking procedure
            self.blackboard.T_homebase_to_dock = inverse_times((hb_translation, hb_rotation), (ds_translation, ds_rotation))
            pose = geometry_msgs.PoseWithCovarianceStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "map"
            pose.pose.pose = transform_to_pose((ds_translation, ds_rotation))
            self._location_publisher.publish(pose)
            rospy.sleep(1) # let things initialise once the pose is sent
            rospy.logdebug("Undock : sent initial pose to initialise navigation")
            self.connected = self.action_client.wait_for_server(rospy.Duration(0.5))
            if not self.connected:
                rospy.logerr("Undock : could not connect to autonomous docking server.")
                self.feedback_message = "Action client failed to connect"
                return py_trees.Status.INVALID
            else:
                goal = gopher_std_msgs.AutonomousDockingGoal()
                goal.command = gopher_std_msgs.AutonomousDockingGoal.UNDOCK
                self.action_client.send_goal(goal)
                self.sent_goal = True

            if self._honk_publisher:
                self._honk_publisher.publish(std_msgs.Empty())

        result = self.action_client.get_result()
        if result:
            if result.value == gopher_std_msgs.AutonomousDockingResult.SUCCESS:
                rospy.logdebug("Undock : " + result.message)
                self.feedback_message = result.message
                return py_trees.Status.SUCCESS
            elif result.value == gopher_std_msgs.AutonomousDockingResult.ABORTED_OBSTACLES:
                rospy.logdebug("Undock : " + result.message)
                self.feedback_message = result.message
                return py_trees.Status.FAILURE
            else:
                rospy.logdebug("Undock : got unknown result value from undocking controller")
                self.feedback_message = "Got unknown result value from undocking controller"
                return py_trees.Status.FAILURE
        else:
            rospy.logdebug("Undock : undocking in progress")
            self.feedback_message = "Undocking in progress"
            return py_trees.Status.RUNNING

    def abort(self, new_status):
        if self.sent_goal:
            action_state = self.action_client.get_state()
            if action_state == GoalStatus.PENDING or action_state == GoalStatus.ACTIVE:
                self.action_client.cancel_goal()


class NotifyComplete(py_trees.Behaviour):
    def __init__(self, name):
        super(NotifyComplete, self).__init__(name)
        self._notify_publisher = rospy.Publisher("~display_notification", gopher_std_msgs.Notification, queue_size=1)

    def update(self):
        # always returns success
        self._notify_publisher.publish(gopher_std_msgs.Notification(led_pattern=gopher_std_msgs.Notification.FLASH_GREEN,
                                                                    button_cancel=gopher_std_msgs.Notification.RETAIN_PREVIOUS,
                                                                    button_confirm=gopher_std_msgs.Notification.RETAIN_PREVIOUS,
                                                                    message="successfully completed action"))
        return py_trees.Status.SUCCESS


class WaitForCharge(py_trees.Behaviour):

    def __init__(self, name):
        super(WaitForCharge, self).__init__(name)
        self._notify_publisher = rospy.Publisher("~display_notification", gopher_std_msgs.Notification, queue_size=1)
        self._battery_subscriber = rospy.Subscriber("~battery", somanet_msgs.SmartBatteryStatus, self.battery_callback)
        self._button_subscriber = rospy.Subscriber("/gopher/buttons/stop", std_msgs.Empty, self.button_callback)
        self.notify_timer = None
        self.button_pressed = False
        self.charge_state = None

    def initialise(self):
        self.send_notification(None)
        # Notifications last for some amount of time before LEDs go back to
        # displaying the battery status. Need to send message repeatedly until
        # the behaviour completes.
        self.notify_timer = rospy.Timer(rospy.Duration(5), self.send_notification)

    def send_notification(self, timer):
        self._notify_publisher.publish(gopher_std_msgs.Notification(led_pattern=gopher_std_msgs.Notification.SOLID_RED,
                                                                    button_cancel=gopher_std_msgs.Notification.RETAIN_PREVIOUS,
                                                                    button_confirm=gopher_std_msgs.Notification.RETAIN_PREVIOUS,
                                                                    message="waiting for charge"))

    def battery_callback(self, msg):
        self.charge_state = msg.charge_state

    def button_callback(self, msg):
        self.button_pressed = True

    def update(self):
        if self.charge_state == somanet_msgs.SmartBatteryStatus.CHARGING or self.button_pressed:
            self._notify_publisher.publish(gopher_std_msgs.Notification(led_pattern=gopher_std_msgs.Notification.CANCEL_CURRENT,
                                                                        button_cancel=gopher_std_msgs.Notification.RETAIN_PREVIOUS,
                                                                        button_confirm=gopher_std_msgs.Notification.RETAIN_PREVIOUS,
                                                                        message="entered charging state or got user input"))
            return py_trees.Status.SUCCESS
        else:
            return py_trees.Status.RUNNING

    def abort(self, new_status):
        if self.notify_timer:
            self.notify_timer.shutdown()


class MoveToGoal(py_trees.Behaviour):
    def __init__(self, name, pose):
        super(MoveToGoal, self).__init__(name)
        self.pose = pose
        self.action_client = None
        self.blackboard = Blackboard()

        self._honk_publisher = None
        try:
            if rospy.get_param("~enable_honks") and rospy.get_param("~moving_honk"):
                honk_topic = rospy.get_param("~moving_honk")
                self._honk_publisher = rospy.Publisher("/gopher/commands/sounds/" + honk_topic, std_msgs.Empty, queue_size=1)
        except KeyError:
            rospy.logwarn("Gopher Deliveries : Could not find param to initialise honks.")
            pass

    def initialise(self):
        self.logger.debug("  %s [MoveToGoal::initialise()]" % self.name)
        self.action_client = actionlib.SimpleActionClient('~move_base', move_base_msgs.MoveBaseAction)

        connected = self.action_client.wait_for_server(rospy.Duration(0.5))
        if not connected:
            rospy.logwarn("MoveToGoal : could not connect with move base.")
            self.action_client = None
        else:
            goal = move_base_msgs.MoveBaseGoal()  # don't yet care about the target
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.pose.position.x = self.pose.x
            goal.target_pose.pose.position.y = self.pose.y
            quaternion = tf.transformations.quaternion_from_euler(0, 0, self.pose.theta)
            goal.target_pose.pose.orientation.x = quaternion[0]
            goal.target_pose.pose.orientation.y = quaternion[1]
            goal.target_pose.pose.orientation.z = quaternion[2]
            goal.target_pose.pose.orientation.w = quaternion[3]
            self.action_client.send_goal(goal)

        if self._honk_publisher:
            self._honk_publisher.publish(std_msgs.Empty())

    def update(self):
        self.logger.debug("  %s [MoveToGoal::update()]" % self.name)
        if self.action_client is None:
            self.feedback_message = "action client couldn't connect"
            return py_trees.Status.INVALID
        result = self.action_client.get_result()
        # self.action_client.wait_for_result(rospy.Duration(0.1))  # < 0.1 is moot here - the internal loop is 0.1
        if result:
            self.feedback_message = "goal reached"
            self.blackboard.traversed_locations.append(self.blackboard.remaining_locations.pop(0))
            return py_trees.Status.SUCCESS
        else:
            self.feedback_message = "moving"
            return py_trees.Status.RUNNING

    def abort(self, new_status):
        # if we have an action client and the current goal has not already
        # succeeded, send a message to cancel the goal for this action client.
        if self.action_client is not None and self.action_client.get_state() != GoalStatus.SUCCEEDED:
            self.action_client.cancel_goal()


class GoHome(py_trees.Behaviour):
    def __init__(self, name):
        super(GoHome, self).__init__(name)

    def update(self):
        self.logger.debug("  %s [GoHome::update()]" % self.name)
        return py_trees.Status.SUCCESS

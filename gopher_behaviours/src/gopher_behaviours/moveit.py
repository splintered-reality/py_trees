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
import nav_msgs.msg as nav_msgs
import geometry_msgs.msg as geometry_msgs
import ar_track_alvar_msgs.msg as ar_track_alvar_msgs
import py_trees
import tf
import rospy
import math
import numpy
from gopher_semantics.map_locations import SemanticLocations
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
    q1nonhom = q1[numpy.ix_([0,1,2],[0,1,2])]
    difft = v * q1nonhom

    return (tuple(numpy.array(difft)[0]), tuple(diffq))

# convert a transform to a human readable string
def human_transform(t):
    return "x translation: " + str(t[0][0]) + "\ny translation: " + str(t[0][1]) + "\n yaw: " + str(numpy.degrees(tf.transformations.euler_from_quaternion(t[1])[2]))

# get the bearing of a transform in the coordinate frame of the transform which
# the given transform is relative to, i.e. given T_homebase_to_dock, this
# function will give you the bearing of the dock from the homebase, in the coordinate frame of the homebase
def transform_bearing(transform):
    # compute the angle between the x axis and the translation vector of the other transform
    xaxis = [1,0]
    # the translation is the position of the other point of interest in the
    # parent frame
    translation = [transform[0][0], transform[0][1]]
    print(translation)
    cross = numpy.cross(xaxis, translation) # the sign tells us whether this is a positive or negative rotation
    # tan formula from https://newtonexcelbach.wordpress.com/2014/03/01/the-angle-between-two-vectors-python-version/
    return numpy.sign(cross) * numpy.arctan2(numpy.linalg.norm(cross), numpy.dot(xaxis,translation))

##############################################################################
# Combined behaviour classes
##############################################################################

class Starting(py_trees.Selector):
    def __init__(self, name):
        undockseq = py_trees.Sequence("Undock", [IsDocked("Check for dock"), Undock("Undock")])
        children = [undockseq,  Unpark("Unpark")]
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

        goal.motion_amount = value
        goal.unsafe = False
        self.action_client.send_goal(goal)
        
    def complete(self):
        result = self.action_client.get_result()
        if not result:
            return False

        return True

    def success(self):
        result = self.action_client.get_result()
        if not result:
            return False
        elif result.value == gopher_std_msgs.SimpleMotionResult.SUCCESS:
            return True
        else: # anything else is a failure state
            return False

    def abort(self, new_status):
        self.action_client.cancel_goal()

class IsDocked(py_trees.Behaviour):
    def __init__(self, name):
        super(IsDocked, self).__init__(name)
        self._battery_subscriber = self._battery_subscriber = rospy.Subscriber("~battery", somanet_msgs.SmartBatteryStatus, self.battery_callback)
        self.docked = None

    def battery_callback(self, msg):
        self.docked = msg.charging_source == somanet_msgs.SmartBatteryStatus.CHARGING_SOURCE_DOCK

    def update(self):
        if self.docked == None: # no message yet
            return py_trees.Status.RUNNING
        elif self.docked: # is docked
            return py_trees.Status.SUCCESS
        else: # is not docked
            return py_trees.Status.FAILURE

class WasDocked(py_trees.Behaviour):
    def __init__(self, name):
        super(WasDocked, self).__init__(name)
        self.blackboard = Blackboard()

    def update(self):
        if self.blackboard.parked:
            return py_trees.Status.FAILURE
        else:
            return py_trees.Status.SUCCESS


class Park(py_trees.Behaviour):
    def __init__(self, name):
        super(Park, self).__init__(name)
        self.rotated = False
        self.tf_listener = tf.TransformListener()
        self.semantic_locations = SemanticLocations()
        self.motion = SimpleMotion()
        self.blackboard = Blackboard()
        self.translated = False
        self.not_unparked = False
        self.homebase = None  # we do delayed retrieval in initialise()

    def initialise(self):
        # if the homebase to dock transform is unset, we didn't do an unparking
        # action - this could mean that the run was started without doing any
        # undock or unpark action
        if self.blackboard.T_homebase_to_dock == None:
            self.not_unparked = True
            return
            
        self.homebase = self.semantic_locations['homebase']
        # assume map frame is 0,0,0 with no rotation; translation of
        # homebase is the position of the homebase specified in semantic
        # locations
        hb_translation = (self.homebase.pose.x, self.homebase.pose.y, 0)
        # quaternion specified by rotating theta radians around the yaw axis
        hb_rotation = tuple(tf.transformations.quaternion_about_axis(self.homebase.pose.theta, (0,0,1)))

        rospy.logdebug("Park : waiting for transform from map to base_link")
        # get the current position of the robot as a transform from map to base link
        self.tf_listener.waitForTransform("map", "base_link", rospy.Time(0), rospy.Duration(15))
        self.start_transform = self.tf_listener.lookupTransform("map", "base_link", rospy.Time(0))

        # get the relative rotation and translation between the homebase and the current position
        T_hb_to_current = inverse_times((hb_translation, hb_rotation), self.start_transform)
        # get the relative rotation between the homebase and the parking location
        T_hb_to_parking = self.blackboard.T_hb_to_parking

        rospy.logdebug("Park : transform from homebase to current location")
        rospy.logdebug(human_transform(T_hb_to_current))
        rospy.logdebug("Park : transform from homebase to parking location")
        rospy.logdebug(human_transform(T_hb_to_parking))

        # create transformstamped objects for the transformations from homebase to the two locations
        t = tf.Transformer(True, rospy.Duration(10))
        cur = geometry_msgs.TransformStamped() # hb_to_current
        cur.header.frame_id = "homebase"
        cur.child_frame_id = "current"
        cur.transform.translation.x = T_hb_to_current[0][0]
        cur.transform.translation.y = T_hb_to_current[0][1]
        cur.transform.translation.z = T_hb_to_current[0][2]

        cur.transform.rotation.x = T_hb_to_current[1][0]
        cur.transform.rotation.y = T_hb_to_current[1][1]
        cur.transform.rotation.z = T_hb_to_current[1][2]
        cur.transform.rotation.w = T_hb_to_current[1][3]
        t.setTransform(cur)

        park = geometry_msgs.TransformStamped() # hb_to_parking
        park.header.frame_id = "homebase"
        park.child_frame_id = "parking"
        park.transform.translation.x = T_hb_to_parking[0][0]
        park.transform.translation.y = T_hb_to_parking[0][1]
        park.transform.translation.z = T_hb_to_parking[0][2]

        park.transform.rotation.x = T_hb_to_parking[1][0]
        park.transform.rotation.y = T_hb_to_parking[1][1]
        park.transform.rotation.z = T_hb_to_parking[1][2]
        park.transform.rotation.w = T_hb_to_parking[1][3]
        t.setTransform(park)

        # Transform from current to parking computed by looking up the transform
        # through the transformer. Current is our current frame of reference, so
        # we know the location of the parking point in that frame. We need to
        # compute the bearing to that point so that we can rotate to it, before
        # moving in its direction.
        T_current_to_parking = t.lookupTransform("current", "parking", rospy.Time(0))

        rospy.logdebug("Park : transform from current location to parking")
        rospy.logdebug(human_transform(T_current_to_parking))

        # straight line distance between current location and parking
        self.distance = numpy.linalg.norm(T_current_to_parking[0][:2])
        rospy.logdebug("Park : straight line distance is {0}".format(self.distance))

        # rotation to perform at the current position to align us with the
        # parking location position so we can drive towards it
        self.rotation_to_parking = transform_bearing(T_current_to_parking)
        # rotation to perform when at the parking location to align us as before
        self.rotation_at_parking = tf.transformations.euler_from_quaternion(T_current_to_parking[1])[2] - self.rotation_to_parking
        # Start execution of the rotation motion which will orient us facing the
        # parking location. Update will check if the motion is complete and
        # whether or not it was successful
        self.motion.execute("rotate", self.rotation_to_parking)
        rospy.logdebug("Park : relative rotation is {0}".format(self.rotation_to_parking))

    def update(self):
        if self.not_unparked:
            self.feedback_message = "transform to parking is undefined - probably no unpark action was performed"
            return py_trees.Status.FAILURE
        # if the motion isn't complete, we're running
        if not self.motion.complete():
            return py_trees.Status.RUNNING
        else:
            # Otherwise, the motion is finished. If either of the motions fail,
            # the behaviour fails.
            if not self.motion.success():
                self.blackboard.unpark_success = False
                return py_trees.Status.FAILURE
            else:
                # The motion successfully completed
                if not self.rotated:
                    # if we weren't rotated, the behaviour completed was a
                    # rotation
                    self.rotated = True
                    # once the rotation is complete, we execute the translation
                    self.motion.execute("translate", self.distance)
                    return py_trees.Status.RUNNING
                else:
                    if not self.translated:
                        # successfully completed the translation - start the
                        # rotation at the parking spot.
                        self.translated = True
                        self.motion.execute("rotate", self.rotation_at_parking)
                        return py_trees.Status.RUNNING
                    else:
                        # if we were rotated and translated, this means that the
                        # rotation at the parking spot succeeded, so all three
                        # actions were successful
                        return py_trees.Status.SUCCESS

    def abort(self, new_status):
        self.motion.abort()
        # set to none to use later as a flag to inidicate not having performed
        # unparking behaviour
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
            
        self.semantic_locations = SemanticLocations()
        self.last_dslam = None
        self.button_pressed = False
        self.homebase = None  # set this in the initialise() step

    def initialise(self):
        self.blackboard.parked = True  # set this to true to indicate that the robot was parked, for the wasdocked behaviour
        # only initialise subscribers when the behaviour starts running
        self.homebase = self.semantic_locations['homebase']
        self._battery_subscriber = self._battery_subscriber = rospy.Subscriber("~battery", somanet_msgs.SmartBatteryStatus, self.battery_callback)
        self._button_subscriber = rospy.Subscriber("/gopher/buttons/go", std_msgs.Empty, self.button_callback)
        self._dslam_subscriber = rospy.Subscriber("/dslam/diagnostics", dslam_msgs.Diagnostics, self.dslam_callback)

        # assume map frame is 0,0,0 with no rotation; translation of homebase is
        # the position of the homebase specified in semantic locations. We need
        # the homebase location regardless of whether dslam is initialised.
        self.hb_translation = (self.homebase.pose.x, self.homebase.pose.y, 0)
        # quaternion specified by rotating theta radians around the yaw axis
        self.hb_rotation = tuple(tf.transformations.quaternion_about_axis(self.homebase.pose.theta, (0,0,1)))
        while not self.last_dslam and not rospy.is_shutdown():
            rospy.logdebug("Unpark : waiting for dslam state update")
            rospy.sleep(2)

        self.dslam_initialised = self.last_dslam.state == dslam_msgs.Diagnostics.WORKING
        if self.dslam_initialised and self.blackboard.unpark_success:
            # if dslam is initialised, the map->base_link transform gives us the
            # actual position of the robot, i.e. the position of the parking
            # spot. Save this as the starting position
            rospy.logdebug("Unpark : waiting for transform from map to base_link")
            self.tf_listener.waitForTransform("map", "base_link", rospy.Time(), rospy.Duration(15))
            self.start_transform = self.tf_listener.lookupTransform("map", "base_link", rospy.Time(0))
        else:
            # save the latest odom message received so that we can use it to compute
            # a transform back to the starting position
            rospy.logdebug("Unpark : waiting for transform from odom to base_link")
            self.tf_listener.waitForTransform("odom", "base_link", rospy.Time(), rospy.Duration(15))
            self.start_transform = self.tf_listener.lookupTransform("odom", "base_link", rospy.Time(0))

    def battery_callback(self, msg):
        self.still_charging = msg.charge_state == somanet_msgs.SmartBatteryStatus.CHARGING

    def button_callback(self, msg):
        self.button_pressed = True
        # when the button is pressed, save the latest odom value so we can get
        # the transform between the start and end poses
        self.end_transform = self.tf_listener.lookupTransform("odom", "base_link", rospy.Time(0))

    def dslam_callback(self, msg):
        self.last_dslam = msg

    def update(self):
        # need to be unplugged before we can move or be moved
        if self.still_charging:
            rospy.logdebug("Unpark : robot is still charging")
            self._notify_publisher.publish(gopher_std_msgs.Notification(led_pattern=gopher_std_msgs.Notification.FLASH_YELLOW))
            # reset the button just in case it was accidentally pressed
            self.button_pressed = False
        else: # was unplugged, should start moving
            if self.dslam_initialised and self.blackboard.unpark_success:
                # if dslam is initialised, we have the map-relative position of
                # the parking spot. Need to compute the relative position of the
                # parking spot to the homebase - this is the difference
                # transform between the parking spot and the homebase
                
                # transform homebase to parking spot
                self.blackboard.T_hb_to_parking = inverse_times((self.hb_translation, self.hb_rotation), self.start_transform)
                rospy.logdebug("Unpark : transform from homebase to parking location")
                print(human_transform(self.blackboard.T_hb_to_parking))

                self.blackboard.unpark_success = True
                return py_trees.Status.SUCCESS
            else:
                rospy.logdebug("Unpark : waiting for go button to be pressed to indicate homebase")
                # otherwise, the robot needs to be guided to the homebase by
                # someone - pressing the go button indicates that the homebase
                # has been reached.
                rospy.logdebug(human_transform(self.tf_listener.lookupTransform("odom", "base_link", rospy.Time(0))))
                if self.button_pressed:
                    self.blackboard.T_hb_to_parking = inverse_times(self.end_transform, self.start_transform)
                    rospy.logdebug("Unpark : transform from homebase to parking location")
                    print(human_transform(self.blackboard.T_hb_to_parking))

                    pose = geometry_msgs.PoseWithCovarianceStamped()
                    pose.header.stamp = rospy.Time.now()
                    pose.header.frame_id = "map"
                    pose.pose.pose.position.x = self.hb_translation[0]
                    pose.pose.pose.position.y = self.hb_translation[1]
                    pose.pose.pose.position.z = self.hb_translation[2]
                    pose.pose.pose.orientation.x = self.hb_rotation[0]
                    pose.pose.pose.orientation.y = self.hb_rotation[1]
                    pose.pose.pose.orientation.z = self.hb_rotation[2]
                    pose.pose.pose.orientation.w = self.hb_rotation[3]
                    self._location_publisher.publish(pose)
                    self.blackboard.unpark_success = True
                    return py_trees.Status.SUCCESS
                self._notify_publisher.publish(gopher_std_msgs.Notification(led_pattern=gopher_std_msgs.Notification.FLASH_PURPLE))
                
        return py_trees.Status.RUNNING
        
class Dock(py_trees.Behaviour):
    """
    Engage the docking behaviour
    """
    def __init__(self, name):
        super(Dock, self).__init__(name)
        self.docking_client = actionlib.SimpleActionClient('~autonomous_docking', gopher_std_msgs.AutonomousDockingAction)
        self.blackboard = Blackboard()
        self.semantic_locations = SemanticLocations()
        self.tf_listener = tf.TransformListener()
        self.interrupted = False # button pressed to interrupt docking procedure?
        self.goal_sent = False # docking goal sent?
        self.motion = SimpleMotion()
        self.not_undocked = False

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

        # if the homebase to dock transform is unset, we didn't do a docking
        # action - this could mean that the run was started without doing any
        # undock or unpark action
        if self.blackboard.T_homebase_to_dock == None:
            self.not_undocked = True
            return

        self.homebase = self.semantic_locations['homebase']
        hb_translation = (self.homebase.pose.x, self.homebase.pose.y, 0)
        hb_rotation = tuple(tf.transformations.quaternion_about_axis(self.homebase.pose.theta, (0,0,1)))

        rospy.logdebug("Dock : waiting for transform from map to base_link")
        self.tf_listener.waitForTransform("map", "base_link", rospy.Time(0), rospy.Duration(15))
        start_transform = self.tf_listener.lookupTransform("map", "base_link", rospy.Time(0))
        T_hb_to_current = inverse_times(start_transform, (hb_translation, hb_rotation))
        T_hb_to_dock = self.blackboard.T_homebase_to_dock
        
        rospy.logdebug("Park : transform from homebase to current location")
        rospy.logdebug(human_transform(T_hb_to_current))
        rospy.logdebug("Park : transform from homebase to docking location")
        rospy.logdebug(human_transform(T_hb_to_dock))

        # create transformstamped objects for the transformations from homebase to the two locations
        t = tf.Transformer(True, rospy.Duration(10))
        cur = geometry_msgs.TransformStamped() # hb_to_current
        cur.header.frame_id = "homebase"
        cur.child_frame_id = "current"
        cur.transform.translation.x = T_hb_to_current[0][0]
        cur.transform.translation.y = T_hb_to_current[0][1]
        cur.transform.translation.z = T_hb_to_current[0][2]

        cur.transform.rotation.x = T_hb_to_current[1][0]
        cur.transform.rotation.y = T_hb_to_current[1][1]
        cur.transform.rotation.z = T_hb_to_current[1][2]
        cur.transform.rotation.w = T_hb_to_current[1][3]
        t.setTransform(cur)

        dock = geometry_msgs.TransformStamped() # hb_to_dock
        dock.header.frame_id = "homebase"
        dock.child_frame_id = "docking"
        dock.transform.translation.x = T_hb_to_dock[0][0]
        dock.transform.translation.y = T_hb_to_dock[0][1]
        dock.transform.translation.z = T_hb_to_dock[0][2]

        dock.transform.rotation.x = T_hb_to_dock[1][0]
        dock.transform.rotation.y = T_hb_to_dock[1][1]
        dock.transform.rotation.z = T_hb_to_dock[1][2]
        dock.transform.rotation.w = T_hb_to_dock[1][3]
        t.setTransform(dock)

        T_current_to_dock = t.lookupTransform("current", "docking", rospy.Time(0))

        rospy.logdebug("Dock : transform from current location to dock")
        rospy.logdebug(human_transform(T_current_to_parking))
        
        self.motion.execute("rotate", transform_bearing(T_current_to_parking))
        
        self.connected = self.docking_client.wait_for_server(rospy.Duration(0.5))
        if not self.connected:
            rospy.logwarn("Dock : could not connect to autonomous docking server.")
        if self._honk_publisher:
            self._honk_publisher.publish(std_msgs.Empty())

    def interrupt_cb(self, msg):
        rospy.logdebug("Dock behaviour got interrupt from button")
        self.interrupted = True
        self.docking_client.cancel_goal()
        self.motion.abort()

    def update(self):
        self.logger.debug("  %s [Dock::update()]" % self.name)
        if self.not_undocked:
            self.feedback_message = "Transform to dock was unset - no undock action was performed"
            return py_trees.Status.FAILURE

        if not self.connected:
            self.feedback_message = "Action client failed to connect"
            return py_trees.Status.INVALID

        if self.interrupted:
            return py_trees.Status.FAILURE

        # running if the rotation motion is still executing
        if not self.motion.complete():
            return py_trees.Status.RUNNING

        # if the rotation motion fails, the behaviour fails
        if not self.motion.success():
            return py_trees.Status.FAILURE
        else:
            if not self.goal_sent:
                # the rotation succeeded, now send the docking goal
                goal = gopher_std_msgs.AutonomousDockingGoal
                goal.command = gopher_std_msgs.AutonomousDockingGoal.DOCK
                self.docking_client.send_goal(goal)
                self.goal_sent = True

        # at this point, we have sent the docking goal - check whether it's
        # succeeded or failed, or if it's still running.
        result = self.docking_client.get_result()
        if result:
            if result.value == gopher_std_msgs.AutonomousDockingResult.SUCCESS:
                self.feedback_message = result.message
                return py_trees.Status.SUCCESS
            elif result.value == gopher_std_msgs.AutonomousDockingResult.ABORTED_OBSTACLES:
                self.feedback_message = result.message
                return py_trees.Status.FAILURE
            else:
                self.feedback_message = "Got unknown result value from docking controller"
                return py_trees.Status.FAILURE
        else:
            self.feedback_message = "Docking in progress"
            return py_trees.Status.RUNNING

    def abort(self, new_status):
        self.motion.abort()
        self.docking_client.cancel_goal()
        # reset the homebase to dock transform to none so we can use it as a
        # flag to indicate that there was no undock performed
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
        self.semantic_locations = SemanticLocations()

        try:
            if rospy.get_param("~enable_honks") and rospy.get_param("~undocking_honk"):
                honk_topic = rospy.get_param("~undocking_honk")
                self._honk_publisher = rospy.Publisher("/gopher/commands/sounds/" + honk_topic, std_msgs.Empty, queue_size=1)
        except KeyError:
            rospy.logwarn("Gopher Deliveries : Could not find param to initialise honks.")
            pass

    def initialise(self):
        self.homebase = self.semantic_locations['homebase']
        self.blackboard.parked = False
        self._ar_long_subscriber = rospy.Subscriber("ar_pose_marker_long_range", ar_track_alvar_msgs.AlvarMarkers, self.ar_cb_long)
        self._ar_short_subscriber = rospy.Subscriber("ar_pose_marker_short_range", ar_track_alvar_msgs.AlvarMarkers, self.ar_cb_short)
        rospy.sleep(rospy.Duration(2)) # sleep to wait for ar marker messages
        self.blackboard.dock_ar_marker_large = self.ar_marker_long
        self.blackboard.dock_ar_markers_small = self.ar_markers_short

        # retrieve information about this docking station
        ds = self.docking_stations.find_docking_stations_with_ar_marker_id(self.ar_marker_long)[0]
        # initialise a transform with the pose of the station
        ds_translation = (ds.pose.x, ds.pose.y, 0)
        # quaternion specified by rotating theta radians around the yaw axis
        ds_rotation = tuple(tf.transformations.quaternion_about_axis(ds.pose.theta, (0,0,1)))

        hb_translation = (self.homebase.pose.x, self.homebase.pose.y, 0)
        hb_rotation = tuple(tf.transformations.quaternion_about_axis(self.homebase.pose.theta, (0,0,1)))

        # publish that information to initialise navigation, and save for the
        # docking procedure
        self.blackboard.T_homebase_to_dock = inverseTimes((hb_translation, hb_rotation), (ds_translation, ds_rotation))
        pose = geometry_msgs.PoseWithCovarianceStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        pose.pose.pose.position.x = ds_translation[0]
        pose.pose.pose.position.y = ds_translation[1]
        pose.pose.pose.position.z = ds_translation[2]
        pose.pose.pose.orientation.x = ds_rotation[0]
        pose.pose.pose.orientation.y = ds_rotation[1]
        pose.pose.pose.orientation.z = ds_rotation[2]
        pose.pose.pose.orientation.w = ds_rotation[3]
        self._location_publisher.publish(pose)
        
        self.connected = self.action_client.wait_for_server(rospy.Duration(0.5))
        if not self.connected:
            rospy.logwarn("Undock : could not connect to autonomous docking server.")
        else:
            goal = gopher_std_msgs.AutonomousDockingGoal
            goal.command = gopher_std_msgs.AutonomousDockingGoal.UNDOCK
            self.action_client.send_goal(goal)

        if self._honk_publisher:
            self._honk_publisher.publish(std_msgs.Empty())

    def ar_cb_long(self, msg):
        # there should only be one longrange marker
        self.ar_marker_long = msg.markers[0].id

    def ar_cb_short(self, msg):
        # can have multiple short range markers
        self.ar_markers_short = [marker.id for marker in msg.markers]

    def update(self):
        self.logger.debug("  %s [Undock::update()]" % self.name)
        if not self.connected:
            self.feedback_message = "Action client failed to connect"
            return py_trees.Status.INVALID

        result = self.action_client.get_result()
        if result:
            if result.value == gopher_std_msgs.AutonomousDockingResult.SUCCESS:
                self.feedback_message = result.message
                return py_trees.Status.SUCCESS
            elif result.value == gopher_std_msgs.AutonomousDockingResult.ABORTED_OBSTACLES:
                self.feedback_message = result.message
                return py_trees.Status.FAILURE
            else:
                self.feedback_message = "Got unknown result value from undocking controller"
                return py_trees.Status.INVALID
        else:
            self.feedback_message = "Unocking in progress"
            return py_trees.Status.RUNNING

    def abort(self, new_status):
        self.action_client.cancel_goal()

class NotifyComplete(py_trees.Behaviour):
    def __init__(self, name):
        super(NotifyComplete, self).__init__(name)
        self._notify_publisher = rospy.Publisher("~display_notification", gopher_std_msgs.Notification, queue_size=1)

    def update(self):
        # always returns success
        self._notify_publisher.publish(gopher_std_msgs.Notification(led_pattern=gopher_std_msgs.Notification.FLASH_GREEN))
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
        self._notify_publisher.publish(gopher_std_msgs.Notification(led_pattern=gopher_std_msgs.Notification.SOLID_RED))

    def battery_callback(self, msg):
        self.charge_state = msg.charge_state

    def button_callback(self, msg):
        self.button_pressed = True

    def update(self):
        if self.charge_state == somanet_msgs.SmartBatteryStatus.CHARGING or self.button_pressed:
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

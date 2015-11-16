#!/usr/bin/env python

from .blackboard import Blackboard
from .navigation import SimpleMotion
from gopher_semantics.semantics import Semantics
import dslam_msgs.msg as dslam_msgs
import geometry_msgs.msg as geometry_msgs
import gopher_configuration
import gopher_std_msgs.msg as gopher_std_msgs
import numpy
import py_trees
import rospy
import somanet_msgs.msg as somanet_msgs
import std_msgs.msg as std_msgs
import tf
import tf_utilities

class Park(py_trees.Behaviour):
    time = None
    def __init__(self, name):
        super(Park, self).__init__(name)
        self.tf_listener = tf.TransformListener()
        self.gopher = gopher_configuration.Configuration()
        self.semantic_locations = Semantics(self.gopher.namespaces.semantics)
        self.motion = SimpleMotion()
        self.blackboard = Blackboard()
        if Park.time == None:
            Park.time = rospy.Time.now()

    def initialise(self):
        self.not_unparked = False
        self.current_transform = None

        # if the homebase to dock transform is unset, we didn't do an unparking
        # action - this could mean that the run was started without doing any
        # undock or unpark action
        if not hasattr(self.blackboard, 'T_hb_to_parking') or self.blackboard.T_hb_to_parking is None:
            self.not_unparked = True
            return

        homebase = self.semantic_locations.semantic_modules['locations']['homebase']
        # assume map frame is 0,0,0 with no rotation; translation of
        # homebase is the position of the homebase specified in semantic
        # locations
        hb_translation = (homebase.pose.x, homebase.pose.y, 0)
        # quaternion specified by rotating theta radians around the yaw axis
        hb_rotation = tuple(tf.transformations.quaternion_about_axis(homebase.pose.theta, (0, 0, 1)))

        self.hb = (hb_translation, hb_rotation) # store so we can use it later
        self.timeout = rospy.Time.now() + rospy.Duration(10)

    def log_parking_location(self, success):
        self.tf_listener.waitForTransform("map", "base_link", rospy.Time(), rospy.Duration(1))
        # do not replace self.current_transform
        current_transform = self.tf_listener.lookupTransform("map", "base_link", rospy.Time(0))
        t = tf.Transformer(True, rospy.Duration(10))
        cur = tf_utilities.transform_to_transform_stamped(current_transform)
        cur.header.frame_id = "map"
        cur.child_frame_id = "current"
        t.setTransform(cur)

        hb = tf_utilities.transform_to_transform_stamped(self.hb)
        hb.header.frame_id = "map"
        hb.child_frame_id = "homebase"
        t.setTransform(hb)

        park = tf_utilities.transform_to_transform_stamped(self.blackboard.T_hb_to_parking)
        park.header.frame_id = "homebase"
        park.child_frame_id = "parking"
        t.setTransform(park)

        T_current_to_parking = t.lookupTransform("current", "parking", rospy.Time(0))
        T_map_to_parking = t.lookupTransform("map", "parking", rospy.Time(0))
        with open("/opt/groot/parkinglog-" + str(Park.time) + ".txt", "a") as f:
            if success:
                f.write("--------------------SUCCESS--------------------\n")
            else:
                f.write("--------------------FAILURE--------------------\n")
            f.write("homebase" + tf_utilities.human_transform(self.hb, oneline=True) + "\n")
            f.write("parking" + tf_utilities.human_transform(T_map_to_parking, oneline=True) + "\n")
            f.write("finishing" + tf_utilities.human_transform(current_transform, oneline=True) + "\n")
            f.write("disparity" + tf_utilities.human_transform(T_current_to_parking, oneline=True) + "\n")

    def update(self):
        if self.not_unparked:
            rospy.logdebug("Park : transform to parking is undefined - probably no unpark action was performed")
            self.feedback_message = "transform to parking is undefined - probably no unpark action was performed"
            return py_trees.Status.FAILURE

        if self.current_transform is None and self.timeout > rospy.Time.now():
            rospy.logdebug("Park : waiting for transform from map to base_link")
            # get the current position of the robot as a transform from map to base link
            try:
                self.current_transform = self.tf_listener.lookupTransform("map", "base_link", rospy.Time(0))
            except tf.Exception:
                rospy.logwarn("Park : map->base_link transform lookup failed with remaining time " + str(rospy.Time.now() - self.timeout))
                self.feedback_message = "waiting for transform from map to base_link"
                return py_trees.Status.RUNNING

            # get the relative rotation and translation between the homebase and the current position
            T_hb_to_current = tf_utilities.inverse_times(self.hb, self.current_transform)
            # get the relative rotation between the homebase and the parking location
            T_hb_to_parking = self.blackboard.T_hb_to_parking

            # create transformstamped objects for the transformations from homebase to the two locations
            t = tf.Transformer(True, rospy.Duration(10))
            cur = tf_utilities.transform_to_transform_stamped(T_hb_to_current)
            cur.header.frame_id = "homebase"
            cur.child_frame_id = "current"
            t.setTransform(cur)

            park = tf_utilities.transform_to_transform_stamped(T_hb_to_parking)
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
            self.rotation_to_parking = tf_utilities.transform_bearing(T_current_to_parking)

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
            next_motion = self.motions_to_execute.pop(0)
            self.motion.execute(next_motion[0], next_motion[1])

        if self.current_transform is None:
            rospy.logdebug("Park : could not get current transform from map to base_link")
            self.feedback_message = "could not get current transform from map to base_link"
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
                self.log_parking_location(False)
                return py_trees.Status.FAILURE
            else:
                # The motion successfully completed. Check if there is another
                # motion to do, and execute it if there is, otherwise return
                # success
                if len(self.motions_to_execute) == 0:
                    rospy.logdebug("Park : successfully completed all motions")
                    self.feedback_message = "successfully completed all motions"
                    self.log_parking_location(True)
                    return py_trees.Status.SUCCESS
                else:
                    next_motion = self.motions_to_execute.pop(0)
                    self.motion.execute(next_motion[0], next_motion[1])
                    rospy.logdebug("Park : waiting for motion to complete")
                    self.feedback_message = "waiting for motion to complete"
                    return py_trees.Status.RUNNING

    def stop(self, new_status):
        self.motion.stop()
        # set to none to use later as a flag to indicate not having performed
        # unparking behaviour. TODO : is this actually necessary?
        # if new_status == py_trees.Status.FAILURE:
        #     self.blackboard.T_hb_to_parking = None


class Unpark(py_trees.Behaviour):

    last_parking = None
    
    def __init__(self, name):
        super(Unpark, self).__init__(name)
        self.config = gopher_configuration.Configuration()
        self._notify_publisher = rospy.Publisher(self.config.topics.display_notification, gopher_std_msgs.Notification, queue_size=1)
        self._location_publisher = rospy.Publisher(self.config.topics.initial_pose, geometry_msgs.PoseWithCovarianceStamped, queue_size=1)

        self.tf_listener = tf.TransformListener()
        self.blackboard = Blackboard()
        # use unpark success variable to allow dslam-based unparking only when
        # dslam actually has an accurate location lock. If we never successfully
        # unparked before, this is unlikely to be the case.
        if not hasattr(self.blackboard, 'unpark_success'):
            self.blackboard.unpark_success = False

        if not hasattr(self.blackboard, 'unpark_first_run'):
            self.blackboard.unpark_first_run = True
        self.semantic_locations = Semantics(self.config.namespaces.semantics)

    def initialise(self):
        rospy.logdebug("Unpark : initialisation")
        self.last_dslam = None
        self.button_pressed = False
        # indicate whether transforms have been setup from map or odom to base link
        self.transform_setup = False
        self.homebase = None  # set this in the initialise() step
        self.end_transform = None
        self.start_transform = None
        self.dslam_initialised = None
        self.lookup_attempts = 0
        self.blackboard.parked = True  # set this to true to indicate that the robot was parked, for the wasdocked behaviour

        # only initialise subscribers when the behaviour starts running
        self.homebase = self.semantic_locations.semantic_modules['locations']['homebase']
        self._battery_subscriber = self._battery_subscriber = rospy.Subscriber("~battery", somanet_msgs.SmartBatteryStatus, self.battery_callback)
        self._button_subscriber = rospy.Subscriber(self.config.buttons.go, std_msgs.Empty, self.button_callback)
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

    # Get the new parking location. The transform received is expected to be the
    # homebase location in the same frame as the start transform. When no
    # parking behaviour has been done before, both transforms are computed from
    # odometry. When dslam is initialised (usually after a single delivery run),
    # both use information from dslam instead. If the new parking location is
    # different enough from the previous one, the location will be changed. This
    # should help prevent the parking location drifting due to slight errors in
    # localisation.
    def update_parking_location(self, homebase_transform):
        if Unpark.last_parking is not None:
            rospy.logdebug("last parking was nonempty - comparing the new transform to see if the parking location moved")
            rospy.logdebug("last parking: " + tf_utilities.human_transform(Unpark.last_parking, oneline=True))
            new_parking = tf_utilities.inverse_times(homebase_transform, self.start_transform)
            rospy.logdebug("new parking: " + tf_utilities.human_transform(new_parking, oneline=True))
            parking_diff = tf_utilities.inverse_times(Unpark.last_parking, new_parking)
            rospy.logdebug("parking diff: " + tf_utilities.human_transform(parking_diff, oneline=True))

            euc_diff = numpy.linalg.norm(parking_diff[0][:2])
            rospy.logdebug("diff distance: " + str(euc_diff))
            # extract yaw difference
            yaw_diff = numpy.degrees(tf.transformations.euler_from_quaternion(parking_diff[1])[2])
            rospy.logdebug("diff yaw: " + str(yaw_diff))

            # if below threshold, keep the last parking location
            if euc_diff < 1 and numpy.absolute(yaw_diff) < 90:
                rospy.logdebug("differences below threshold. keeping last parking location")
                new_parking = Unpark.last_parking
            else:
                rospy.loginfo("UnPark : Difference from last parking location was above threshold. Parking location modified.")
        else:
            new_parking = tf_utilities.inverse_times(homebase_transform, self.start_transform)
            rospy.logdebug("no previous parking location. Initialising to " + tf_utilities.human_transform(new_parking, oneline=True))

        rospy.logdebug("new parking is " + str(new_parking))
        return new_parking

    def update(self):
        if not self.transform_setup:
            rospy.logdebug("Unpark : parking location transform not set up yet")
            if not self.last_dslam:  # no message from dslam yet
                rospy.logdebug("Unpark : no message from dslam yet")
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
                self.dslam_initialised = True

            if self.dslam_initialised and self.blackboard.unpark_success:
                # if dslam is initialised, the map->base_link transform gives us the
                # actual position of the robot, i.e. the position of the parking
                # spot. Save this as the starting position
                rospy.logdebug("Unpark : waiting for transform from map to base_link")
                try:
                    self.start_transform = self.tf_listener.lookupTransform("map", "base_link", rospy.Time(0))
                    self.transform_setup = True
                    rospy.logdebug("Unpark : got map transform")
                except tf.Exception:
                    rospy.logdebug("Unpark : didn't get map transform")
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
                    rospy.logdebug("Unpark : got odom transform")
                except tf.Exception:
                    rospy.logdebug("Unpark : didn't get odom transform")
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
            rospy.logdebug("Unpark : wasn't plugged in")
            if self.dslam_initialised and self.blackboard.unpark_success:
                rospy.logdebug("Unpark : dslam is initialised, and successfully unparked before")
                # if dslam is initialised, we have the map-relative position of
                # the parking spot. Need to compute the relative position of the
                # parking spot to the homebase - this is the difference
                # transform between the parking spot and the homebase. We also
                # check how much the parking spots differ. If the difference is
                # small, we retain the parking spot that was used before to
                # prevent drift in the parking locations.
                new_parking = self.update_parking_location((self.hb_translation, self.hb_rotation))
                Unpark.last_parking = new_parking
                self.blackboard.T_hb_to_parking = new_parking
                self.blackboard.unpark_success = True
                self.feedback_message = "successfully unparked"
                return py_trees.Status.SUCCESS
            else:
                rospy.logdebug("Unpark : waiting for go button to be pressed to indicate homebase")
                self.feedback_message = "waiting for go button to be pressed to indicate homebase"
                # otherwise, the robot needs to be guided to the homebase by
                # someone - pressing the go button indicates that the homebase
                # has been reached.
                if self.button_pressed:
                    rospy.logdebug("Unpark : button was pressed")
                    if self.end_transform is None:
                        rospy.logerr("Unpark : did not get a homebase transform")
                        self.feedback_message = "did not get a homebase transform"
                        return py_trees.Status.FAILURE

                    # the homebase is defined by the end transform, so we use
                    # that to get the new parking location.
                    new_parking = self.update_parking_location(self.end_transform)
                    Unpark.last_parking = new_parking
                    self.blackboard.T_hb_to_parking = new_parking
                    rospy.logdebug(self.blackboard.T_hb_to_parking)
                    pose = geometry_msgs.PoseWithCovarianceStamped()
                    pose.header.stamp = rospy.Time.now()
                    pose.header.frame_id = "map"
                    pose.pose.pose = tf_utilities.transform_to_pose((self.hb_translation, self.hb_rotation))
                    self._location_publisher.publish(pose)
                    self.blackboard.unpark_success = True
                    self.feedback_message = "successfully unparked"
                    return py_trees.Status.SUCCESS
                else:
                    rospy.logdebug("Unpark : still waiting on button")
                    # publish notification request to flash and turn on the go button led
                    self._notify_publisher.publish(gopher_std_msgs.Notification(led_pattern=gopher_std_msgs.Notification.FLASH_PURPLE,
                                                                                button_confirm=gopher_std_msgs.Notification.BUTTON_ON,
                                                                                button_cancel=gopher_std_msgs.Notification.RETAIN_PREVIOUS,
                                                                                message="Unpark : waiting for go button press to continue"))
                    return py_trees.Status.RUNNING

        return py_trees.Status.RUNNING

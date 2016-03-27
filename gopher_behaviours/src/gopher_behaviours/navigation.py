#
# License: Yujin
#
##############################################################################
# Description
##############################################################################

"""
.. module:: navigation
   :platform: Unix
   :synopsis: Navigation related behaviours

Oh my spaghettified magnificence,
Bless my noggin with a tickle from your noodly appendages!

----

"""

##############################################################################
# Imports
##############################################################################

import actionlib
import actionlib_msgs.msg as actionlib_msgs
import elf_msgs.msg as elf_msgs
import geometry_msgs.msg as geometry_msgs
import gopher_configuration
import gopher_navi_msgs.msg as gopher_navi_msgs
import gopher_std_msgs.msg as gopher_std_msgs
import gopher_std_msgs.srv as gopher_std_srvs
import math
import move_base_msgs.msg as move_base_msgs
import nav_msgs.msg as nav_msgs
import py_trees
import rospy
import tf
import tf2_geometry_msgs
import tf2_ros

from . import ar_markers
from . import interactions

##############################################################################
# Behaviour Factories
##############################################################################


def create_homebase_teleport(name="Homebase Teleport"):
    behaviour = Teleport(
        name=name,
        goal=gopher_navi_msgs.TeleportGoal(location="homebase", special_effects=True)
    )
    return behaviour


def create_elf_localisation_check_behaviour(name="Localised?"):
    """
    Hooks up a subscriber to the elf to check if everything is
    nice and localised.
    """
    gopher = gopher_configuration.Configuration(fallback_to_defaults=True)
    behaviour = py_trees.CheckSubscriberVariable(
        name=name,
        topic_name=gopher.topics.elf_status,
        topic_type=elf_msgs.ElfLocaliserStatus,
        variable_name="status",
        expected_value=elf_msgs.ElfLocaliserStatus.STATUS_WORKING,
        fail_if_no_data=True,
        fail_if_bad_comparison=True,
        monitor_continuously=True
    )
    return behaviour


def create_elf_localisation_to_blackboard_behaviour(
        name="ElfToBlackboard",
        blackboard_variables={"elf_localisation_status", None}
):
    """
    Hooks up a subscriber to the elf and transfers the status
    message to the blackboard.
    """
    gopher = gopher_configuration.Configuration(fallback_to_defaults=True)

    behaviour = py_trees.SubscriberToBlackboard(
        name=name,
        topic_name=gopher.topics.elf_status,
        topic_type=elf_msgs.ElfLocaliserStatus,
        blackboard_variables=blackboard_variables,
        clearing_policy=py_trees.common.ClearingPolicy.NEVER
    )
    return behaviour


def create_odom_pose_to_blackboard_behaviour(
    name="OdomToBlackboard",
    blackboard_variables={"odom", None}
):
    """
    Hooks up a subscriber and transfers the odometry topic to the blackboard.

    :param str name: behaviour name
    :param str blackboard_variable_name: name to write the message to
    :returns: the behaviour
    :rtype: subscribers.CheckSubscriberVariable
    """
    gopher = gopher_configuration.Configuration(fallback_to_defaults=True)

    behaviour = py_trees.SubscriberToBlackboard(
        name,
        topic_name=gopher.topics.odom,
        topic_type=nav_msgs.Odometry,
        blackboard_variables=blackboard_variables
    )
    return behaviour


def create_map_pose_to_blackboard_behaviour(
    name="PoseToBlackboard",
    blackboard_variables={"pose", None}
):
    """
    Hooks up a subscriber and transfers the pose topic to the blackboard.

    :param str name: behaviour name
    :param str blackboard_variable_name: name to write the message to
    :returns: the behaviour
    :rtype: subscribers.CheckSubscriberVariable
    """
    gopher = gopher_configuration.Configuration(fallback_to_defaults=True)

    behaviour = py_trees.SubscriberToBlackboard(
        name,
        topic_name=gopher.topics.pose,
        topic_type=geometry_msgs.PoseWithCovarianceStamped,
        blackboard_variables=blackboard_variables
    )
    return behaviour


def create_elf_pose_to_blackboard_behaviour(
    name="Elf Pose To Blackboard",
    blackboard_variables={"pose", None}
):
    """
    Hooks up a subscriber and transfers the elf pose to the blackboard.

    :param str name: behaviour name
    :param str blackboard_variable_name: name to write the message to
    :returns: the behaviour
    :rtype: subscribers.CheckSubscriberVariable
    """
    gopher = gopher_configuration.Configuration(fallback_to_defaults=True)

    behaviour = py_trees.SubscriberToBlackboard(
        name,
        topic_name=gopher.topics.elf_status,
        topic_type=elf_msgs.ElfLocaliserStatus,
        blackboard_variables=blackboard_variables
    )
    return behaviour

##############################################################################
# Simple Motion Behaviour
##############################################################################


class SimpleMotion(py_trees.Behaviour):
    """
    Interface to the simple motions controller.
    """
    def __init__(self, name="simple_motions",
                 motion_type=gopher_std_msgs.SimpleMotionGoal.MOTION_ROTATE,
                 motion_amount=0,
                 unsafe=False,
                 keep_going=True,
                 fail_if_complete=False,
                 live_dangerously=False
                 ):
        """
        :param str name: behaviour name
        :param str motion_type: rotation or translation (from gopher_std_msgs.SimpleMotionGoal, MOTION_ROTATE or MOTION_TRANSLATE)
        :param double motion_amount: how far the rotation (radians) or translation (m) should be
        :param bool unsafe: flag if you want the motion to be unsafe, i.e. not use the sensors
        :param bool keep_going: flag if you want the motion to return success in case the action aborts
        :param bool fail_if_complete: flag if you want the motion to return failure in case the motion is completed
        :param bool live_dangeorusly: do not worry about checking the action client is connected

        The ``keep_going`` flag is useful if you are attempting to rotate the robot out of harms way, but don't mind
        if it doesn't make the full specified rotation - this is oft used in navigation recovery style behaviours.

        The ``fail_if_complete`` flag is useful if you are expecting something to happen before the rotation end, i.e.
        if it gets to the end, it should be considered a failure - this could be used in a scanning rotation where it
        is looking for a landmark in the environment. Note that this cannot be achieved by the py_trees invert decorator,
        since that would also invert failed aborts.

        The ``live_dangerously`` flag is useful if you're dynamically creating and adding children in the middle of
        tick_tock. Just makes sure you have your checks done by a higher level pastafarian.
        """
        super(SimpleMotion, self).__init__(name)
        self.gopher = None
        self.action_client = None
        self.sent_goal = False
        self.goal = gopher_std_msgs.SimpleMotionGoal()
        self.goal.motion_type = motion_type
        self.goal.motion_amount = motion_amount
        self.goal.unsafe = unsafe
        self.keep_going = keep_going
        self.fail_if_complete = fail_if_complete
        self.live_dangerously = live_dangerously

    def setup(self, timeout):
        """
        Wait for the action server to come up. Note that ordinarily you do not
        need to call this directly since the :py:function:initialise::`initialise`
        method will call this for you on the first entry into the behaviour, but it
        does so with an insignificant timeout. This however is relying on the
        assumption that the underlying system is up and running before the
        behaviour is started, so this method provides a means for a higher level
        pastafarian to wait for the components to fall into place first.

        :param double timeout: time to wait (0.0 is blocking forver)
        :returns: whether it timed out waiting for the server or not.
        :rtype: boolean
        """
        self.logger.debug("  %s [SimpleMotion::setup()]" % self.name)
        if not self.action_client:
            self.gopher = gopher_configuration.Configuration()
            self.action_client = actionlib.SimpleActionClient(
                self.gopher.actions.simple_motion_controller,
                gopher_std_msgs.SimpleMotionAction
            )
            if timeout is not None:
                if not self.action_client.wait_for_server(rospy.Duration(timeout)):
                    # replace with a py_trees exception!
                    self.logger.error("  %s [SimpleMotion::setup()] could not connect to the docking action server" % self.name)
                    rospy.logerr("Behaviour [%s" % self.name + "] could not connect to the simple motions action server [%s]" % self.__class__.__name__)
                    self.action_client = None
                    return False
            # else just assume it's working, maybe None should be handled like infinite blocking
        return True

    def initialise(self):
        self.logger.debug("  %s [SimpleMotion::initialise()]" % self.name)
        self.sent_goal = False

    def update(self):
        self.logger.debug("  %s [SimpleMotion::update()]" % self.name)
        if not self.action_client:
            if self.live_dangerously:
                self.setup(None)
            else:
                self.feedback_message = "no action client, did you call setup() on your tree?"
                return py_trees.Status.FAILURE
        # pity there is no 'is_connected' api like there is for c++
        if not self.sent_goal:
            self.action_client.send_goal(self.goal)
            self.sent_goal = True
            self.feedback_message = "sent goal to the action server"
            return py_trees.Status.RUNNING
        if self.action_client.get_state() == actionlib_msgs.GoalStatus.ABORTED:
            if self.keep_going:
                self.feedback_message = "simple motion aborted, but we keep on marching forward"
                return py_trees.Status.SUCCESS
            else:
                self.feedback_message = "simple motion aborted"
                return py_trees.Status.FAILURE
        result = self.action_client.get_result()
        if result:
            self.feedback_message = "goal reached"
            if self.fail_if_complete:
                return py_trees.Status.FAILURE
            else:
                return py_trees.Status.SUCCESS
        else:
            self.feedback_message = "moving"
            return py_trees.Status.RUNNING

    def terminate(self, new_status):
        # if we have an action client and the current goal has not already
        # succeeded, send a message to cancel the goal for this action client.
        # if self.action_client is not None and self.action_client.get_state() != actionlib_msgs.GoalStatus.SUCCEEDED:
        self.logger.debug("  %s [SimpleMotions.terminate()][%s->%s]" % (self.name, self.status, new_status))
        if self.action_client is not None and self.sent_goal:
            motion_state = self.action_client.get_state()
            if (motion_state == actionlib_msgs.GoalStatus.PENDING) or (motion_state == actionlib_msgs.GoalStatus.ACTIVE):
                self.action_client.cancel_goal()
        self.sent_goal = False


##############################################################################
# Move Base Behaviour
##############################################################################


class MoveIt(py_trees.Behaviour):
    """
    Simplest kind of move it possible. Just connects to the move base action
    and runs that, nothing else.

    Dont ever give up is our default action for moving around until we decide otherwise.
    Set it true here, and later we can disable it here and set it for the few specific
    instances that require it. That will save us from having to do an exhaustive search
    and destroy later.

    .. todo:: decouple the notify service from this class
    """
    def __init__(self, name, pose=geometry_msgs.Pose2D(), dont_ever_give_up=True):
        """
        :param str name: name of the behaviour
        :param geometry_msgs/Pose2D pose: target pose
        :param bool dont_ever_give_up: keep trying, even if it aborted
        """
        super(MoveIt, self).__init__(name)
        self.pose = pose
        self.gopher = None
        self.action_client = None
        self.notify_service = None
        self.dont_ever_give_up = dont_ever_give_up
        self.goal = move_base_msgs.MoveBaseGoal()
        self.goal.target_pose.header.frame_id = "map"
        self.goal.target_pose.pose.position.x = self.pose.x
        self.goal.target_pose.pose.position.y = self.pose.y
        quaternion = tf.transformations.quaternion_from_euler(0, 0, self.pose.theta)
        self.goal.target_pose.pose.orientation.x = quaternion[0]
        self.goal.target_pose.pose.orientation.y = quaternion[1]
        self.goal.target_pose.pose.orientation.z = quaternion[2]
        self.goal.target_pose.pose.orientation.w = quaternion[3]

    def setup(self, timeout):
        """
        Wait for the action server to come up. Note that ordinarily you do not
        need to call this directly since the :py:function:initialise::`initialise`
        method will call this for you on the first entry into the behaviour, but it
        does so with an insignificant timeout. This however is relying on the
        assumption that the underlying system is up and running before the
        behaviour is started, so this method provides a means for a higher level
        pastafarian to wait for the components to fall into place first.

        :param double timeout: time to wait (0.0 is blocking forever)
        :returns: whether it timed out waiting for the server or not.
        :rtype: boolean
        """
        if not self.action_client:
            self.gopher = gopher_configuration.Configuration()
            self.action_client = actionlib.SimpleActionClient(
                self.gopher.actions.move_base,
                move_base_msgs.MoveBaseAction
            )
            if not self.action_client.wait_for_server(rospy.Duration(timeout)):
                rospy.logerr("Behaviour [%s" % self.name + "] could not connect to the move base action server [%s]" % self.__class__.__name__)
                self.action_client = None
                return False
            try:
                rospy.wait_for_service(self.gopher.services.notification, timeout)
            except rospy.ROSException:
                rospy.logerr("Behaviour [%s" % self.name + "] could not connect to the notifications server [%s]" % self.__class__.__name__)
                self.action_client = None
                return False
            except rospy.ROSInterruptException:
                self.action_client = None
                return False
            self.notify_service = rospy.ServiceProxy(self.gopher.services.notification, gopher_std_srvs.Notify)
        return True

    def initialise(self):
        self.logger.debug("  %s [MoveIt::initialise()]" % self.name)
        if not self.action_client:
            if not self.setup(timeout=0.1):
                return
        self.action_client.send_goal(self.goal)

    def update(self):
        self.logger.debug("  %s [MoveIt::update()]" % self.name)
        if self.action_client is None:
            self.feedback_message = "action client couldn't connect"
            return py_trees.Status.FAILURE
        if self.action_client.get_state() == actionlib_msgs.GoalStatus.ABORTED:
            if self.dont_ever_give_up:
                self.feedback_message = "move base aborted, but we dont ever give up...tallyho!"
                # this will light up for the default period of the notifier (typically 10s)
                self.notify_service(
                    notification=gopher_std_msgs.Notification(
                        led_pattern=gopher_std_msgs.LEDStrip(led_strip_pattern=self.gopher.led_patterns.dab_dab_hae),
                        message=self.feedback_message
                    )
                )
                self.action_client.send_goal(self.goal)
                return py_trees.Status.RUNNING
            else:
                self.feedback_message = "move base aborted"
                return py_trees.Status.FAILURE

        result = self.action_client.get_result()
        if result:
            self.feedback_message = "goal reached"
            return py_trees.Status.SUCCESS
        else:
            self.feedback_message = "moving"
            return py_trees.Status.RUNNING

    def terminate(self, new_status):
        """
        If we have an action client and the current goal has not already
        succeeded, send a message to cancel the goal for this action client.
        """
        if self.action_client is None:
            return
        motion_state = self.action_client.get_state()
        if (motion_state == actionlib_msgs.GoalStatus.PENDING) or (motion_state == actionlib_msgs.GoalStatus.ACTIVE):
            self.action_client.cancel_goal()

##############################################################################
# Teleport Behaviour
##############################################################################


class Teleport(py_trees.Behaviour):
    """
    This is a gopher teleport behaviour that lets you re-initialise the
    gopher across semantic worlds and locations, or just specific poses on maps.
    """
    def __init__(self, name, goal):
        """
        The user should prepare the goal as there are quite a few ways that the
        goal message can be configured (see the comments in the msg file or
        just the help descriptions for the ``gopher_teleport` command line
        program for more information).

        :param gopher_navi_msgs.TeleportGoal goal: a suitably configured goal message.
        """
        super(Teleport, self).__init__(name)
        self.goal = goal
        self.action_client = None
        self.gopher = gopher_configuration.Configuration()

    def initialise(self):
        self.logger.debug("  %s [Teleport::initialise()]" % self.name)
        self.action_client = actionlib.SimpleActionClient(self.gopher.actions.teleport, gopher_navi_msgs.TeleportAction)
        # should not have to wait as this will occur way after the teleport server is up
        connected = self.action_client.wait_for_server(rospy.Duration(0.5))
        if not connected:
            rospy.logwarn("Teleport : behaviour could not connect with the teleport server.")
            self.action_client = None
            # we catch the failure in the first update() call
        else:
            self.action_client.send_goal(self.goal)

    def update(self):
        self.logger.debug("  %s [Teleport::update()]" % self.name)
        if self.action_client is None:
            self.feedback_message = "failed, action client not connected."
            return py_trees.Status.FAILURE
        result = self.action_client.get_result()
        # self.action_client.wait_for_result(rospy.Duration(0.1))  # < 0.1 is moot here - the internal loop is 0.1
        if result is not None:
            if result.value == gopher_navi_msgs.TeleportResult.SUCCESS:
                self.feedback_message = "success"
                return py_trees.Status.SUCCESS
            else:
                self.feedback_message = result.message
                return py_trees.Status.FAILURE
        else:
            self.feedback_message = "waiting for teleport to init pose and clear costmaps"
            return py_trees.Status.RUNNING

    def stop(self, new_status):
        # if we have an action client and the current goal has not already
        # succeeded, send a message to cancel the goal for this action client.
        if self.action_client is not None and self.action_client.get_state() != actionlib_msgs.GoalStatus.SUCCEEDED:
            self.action_client.cancel_goal()

##############################################################################
# Goal Finishing Behaviour
##############################################################################


class GoalFinishing(py_trees.Behaviour):
    """
    Simple goal finishing behaviour; just connects to the goal finishing action and runs that, nothing else.

    :param geometry_msgs/Pose2D goal_pose  the goal pose (relative to the map frame) the robot shall finish at
    """
    def __init__(self, name, goal_pose, distance_threshold=0.1, timeout=50.0):
        """
        The user should prepare the goal as there are quite a few ways that the
        goal message can be configured (see the comments in the msg file or
        just the help descriptions for the ``gopher_teleport` command line
        program for more information).

        :param gopher_navi_msgs.TeleportGoal goal: a suitably configured goal message.
        """
        super(GoalFinishing, self).__init__(name)
        self.action_client = None
        self.goal = None
        self.goal_pose = goal_pose
        self.config = gopher_configuration.Configuration()

        # parameters for re-trying
        self._distance_threshold = distance_threshold
        self._timeout = rospy.Duration(timeout)
        self._approach_timeout = rospy.Duration(0.7 * timeout)
        self._timeout = rospy.Duration(timeout)
        self._time_finishing_start = None

    def initialise(self):
        self.logger.debug("  %s [GoalFinishing::initialise()]" % self.name)
        self.action_client = actionlib.SimpleActionClient(self.config.actions.goal_finishing,
                                                          gopher_navi_msgs.GoalFinishingAction)

        # prepare the goal
        self.goal = gopher_navi_msgs.GoalFinishingGoal()
        pose_stamped = geometry_msgs.PoseStamped()
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.frame_id = "map"
        pose_stamped.pose.position.x = self.goal_pose.x
        pose_stamped.pose.position.y = self.goal_pose.y
        pose_stamped.pose.position.z = 0.0
        quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, self.goal_pose.theta)
        pose_stamped.pose.orientation.x = quaternion[0]
        pose_stamped.pose.orientation.y = quaternion[1]
        pose_stamped.pose.orientation.z = quaternion[2]
        pose_stamped.pose.orientation.w = quaternion[3]
        # convert goal pose from "map" into the "odom" frame, since this one is more reliable for short-distances
        tf_buffer = tf2_ros.Buffer(rospy.Duration(10.0))
        unused_tf_listener = tf2_ros.TransformListener(tf_buffer)
        try:
            # origin frame hard-coded since goal_pose is not stamped
            transform_map_odom = tf_buffer.lookup_transform("odom",
                                                            "map",
                                                            rospy.Time(0),
                                                            rospy.Duration(0.1))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.goal = None
            self.action_client = None
            rospy.logwarn("GoalFinishing : Could not convert the goal pose from the 'map' into 'odom' frame.")
            return
        pose_transformed = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform_map_odom)
        self.goal.pose = pose_transformed
        self.goal.align_on_failure = False

        # should not have to wait as this will occur way after the teleport server is up
        connected = self.action_client.wait_for_server(rospy.Duration(0.5))
        if not connected:
            rospy.logwarn("GoalFinishing : behaviour could not connect with the goal finisher server.")
            self.action_client = None
            # we catch the failure in the first update() call
        else:
            self.action_client.send_goal(self.goal)

        self._time_finishing_start = rospy.Time.now()
        self._final_goal_sent = False

    def update(self):
        self.logger.debug("  %s [GoalFinishing::update()]" % self.name)

        if self.action_client is None:
            self.feedback_message = "failed, action client not connected."
            return py_trees.Status.FAILURE

        current_finishing_duration = rospy.Time.now() - self._time_finishing_start

        result = self.action_client.get_result()
        if result is not None:
            if result.value == gopher_navi_msgs.GoalFinishingResult.SUCCESS:
                self.feedback_message = "success"
                return py_trees.Status.SUCCESS
            else:

                if (result.goal_distance > self._distance_threshold):
                    if (current_finishing_duration > self._approach_timeout) and\
                       (current_finishing_duration <= self._timeout):
                        rospy.logwarn("GoalFinishing: Failed to approach goal in time. Will try to align at least.")
                        self.goal.align_on_failure = True
                    elif (current_finishing_duration > self._timeout):
                        rospy.logwarn("GoalFinishing: Failed to align in time.")
                        self.feedback_message = "failure, but ignoring"
                        return py_trees.Status.SUCCESS
                    else:
                        self.goal.align_on_failure = False
                    # re-try
                    self.action_client.send_goal(self.goal)
                    self.feedback_message = "waiting for goal finisher to finish us"
                    return py_trees.Status.RUNNING
                else:
                    if (current_finishing_duration > self._timeout):
                        rospy.logwarn("GoalFinishing: Close enough, but failed to align in time.")
                        self.feedback_message = "failure, but ignoring"
                        return py_trees.Status.SUCCESS
                    else:
                        # try to align
                        rospy.logwarn("GoalFinishing: Close enough. Will try to align.")
                        self.goal.align_on_failure = True
                        self.action_client.send_goal(self.goal)
                        self.feedback_message = "waiting for goal finisher to finish us"
                        return py_trees.Status.RUNNING
        else:
            if (current_finishing_duration > self._approach_timeout) and\
               (current_finishing_duration <= self._timeout) and not self.goal.align_on_failure:
                # send a new goal, what will preempt the last one
                rospy.logwarn("GoalFinishing: Tried long enough to approach. Forcing the switch to trying to align.")
                self.goal.align_on_failure = True
                self.action_client.send_goal(self.goal)
                return py_trees.Status.RUNNING
            elif (current_finishing_duration > self._timeout):
                rospy.logwarn("GoalFinishing: Time's up for aligning with the goal.")
                self.action_client.cancel_goal()
                self.feedback_message = "failure, but ignoring"
                return py_trees.Status.SUCCESS
            else:
                self.feedback_message = "waiting for goal finisher to finish us"
                return py_trees.Status.RUNNING

    def stop(self, new_status):
        # if we have an action client and the current goal has not already
        # succeeded, send a message to cancel the goal for this action client.
        if self.action_client is not None and self.action_client.get_state() != actionlib_msgs.GoalStatus.SUCCEEDED:
            self.action_client.cancel_goal()

##############################################################################
# Pose Initialisation w/ ELF Behaviour
##############################################################################


class ElfInitialisation(py_trees.Sequence):
    """
    This class implements the sequence of initialising the robot's pose
    using means from the hint providers in the elf localisation framework.

    Blackboard Variables:

     - auto_init_failed     (w) [bool]      : signal to others that the initialisation failed.
    """
    def __init__(self, name):
        """
        Put together the pose intialisation sequence
        """
        super(ElfInitialisation, self).__init__(name)
        self.gopher = gopher_configuration.Configuration()

        # Behaviours
        ar_markers_on = ar_markers.ControlARMarkerTracker("AR Markers On", self.gopher.topics.ar_tracker_long_range, True)
        ar_markers_off = ar_markers.ControlARMarkerTracker("AR Markers Off", self.gopher.topics.ar_tracker_long_range, False)
        ar_markers_off_two = ar_markers.ControlARMarkerTracker("AR Markers Off", self.gopher.topics.ar_tracker_long_range, False)
        # the worker components
        check_elf_status = py_trees.CheckSubscriberVariable(
            name="Check ELF State",
            topic_name=self.gopher.topics.elf_status,
            topic_type=elf_msgs.ElfLocaliserStatus,
            variable_name="status",
            expected_value=elf_msgs.ElfLocaliserStatus.STATUS_WORKING,
            fail_if_no_data=True,
            fail_if_bad_comparison=True,
            clearing_policy=py_trees.common.ClearingPolicy.NEVER
        )
        scanning = py_trees.Selector("Scanning")
        confirmation = py_trees.Sequence("Confirmation")
        notify_done = interactions.SendNotification(
            "Celebrate",
            led_pattern=gopher_std_msgs.LEDStrip.AROUND_RIGHT_GREEN,
            sound=self.gopher.sounds.done,
            message="intialised pose"
        )
        celebrate = py_trees.timers.Timer("Take Time to Celebrate", 2.0)
        rotate_with_timeout = py_trees.composites.Sequence("Rotate w/ Timeout")
        # always succeed so we can go on to the timer
        rotate_ad_nauseum = py_trees.meta.failure_is_success(SimpleMotion(name="Rotate Ad Nauseum", motion_amount=(0.25 * math.pi)))
        # poor man's timeout since it comes after incremental rotations
        timeout = py_trees.meta.success_is_failure(py_trees.timers.Timer("Fail on Timeout", 5.0))
        timed_out = py_trees.composites.Sequence("Timed Out")
        write_auto_initialisation_failed = py_trees.blackboard.SetBlackboardVariable("Flag AutoInit Failed", "auto_init_failed", True)

        self.add_child(ar_markers_on)
        self.add_child(scanning)
        scanning.add_child(confirmation)
        confirmation.add_child(check_elf_status)
        confirmation.add_child(notify_done)
        notify_done.add_child(celebrate)
        scanning.add_child(rotate_with_timeout)
        rotate_with_timeout.add_child(rotate_ad_nauseum)
        rotate_with_timeout.add_child(timeout)
        scanning.add_child(timed_out)
        timed_out.add_child(write_auto_initialisation_failed)
        timed_out.add_child(py_trees.meta.success_is_failure(ar_markers_off_two))  # cleaning up, just make sure we send fail from here

#         timeout = py_trees.timers.create_timeout_subtree(
#             name="Scanning",
#             behaviour=searching,
#             timeout=1.0,
#             timeout_behaviour=ar_markers_off_two
#         )

        self.add_child(ar_markers_off)

#     def initialise(self):
#         """
#         Initialise the pose intialisation sequence
#         """
#         super(ElfInitialisation, self).initialise()
#
#     def stop(self, new_status=py_trees.Status.INVALID):
#         """
#         Stop the pose intialisation sequence
#         """
#         super(ElfInitialisation, self).stop(new_status)

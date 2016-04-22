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
"""

##############################################################################
# Imports
##############################################################################

import actionlib
import actionlib_msgs.msg as actionlib_msgs
import functools
import geometry_msgs.msg as geometry_msgs
import gopher_configuration
import gopher_navi_msgs.msg as gopher_navi_msgs
import gopher_std_msgs.msg as gopher_std_msgs
import gopher_std_msgs.srv as gopher_std_srvs
import move_base_msgs.msg as move_base_msgs
import operator
import nav_msgs.msg as nav_msgs
import py_trees
import rocon_console.console as console
import rospy
import tf
import tf2_geometry_msgs
import tf2_ros

from . import interactions
from . import transform_utilities

##############################################################################
# Behaviour Factories
##############################################################################


def create_teleop_homebase_teleport_subtree(name="Teleop & Teleport"):
    """
    Simple teleop and teleport sequence. Usually you would like to
    add another behaviour to the end of the sequence, e.g.
    py_trees.behaviours.Running if you're waiting for an interrupt
    at a higher priority or a timer to hang around and show the
    teleport for a reasonable duration.
    """
    gopher = gopher_configuration.Configuration(fallback_to_defaults=True)
    teleop_and_teleport = py_trees.composites.Sequence(name)
    teleop_and_teleport.blackbox_level = py_trees.common.BlackBoxLevel.DETAIL
    teleop_to_homebase = py_trees.composites.Parallel(
        name="Teleop to Homebase",
        policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE
    )
    flash_i_need_help = interactions.Notification(
        name='Flash Help Me',
        message='waiting for button press to continue',
        led_pattern=gopher.led_patterns.humans_i_need_help,
        button_confirm=gopher_std_msgs.Notification.BUTTON_ON,
        button_cancel=gopher_std_msgs.Notification.RETAIN_PREVIOUS,
        duration=gopher_std_srvs.NotifyRequest.INDEFINITE
    )
    # usually have the button event handler/blackboard combo and that is less expensive than the subscriber method
    wait_for_go_button_press = py_trees.blackboard.WaitForBlackboardVariable(
        name="Wait for Go Button",
        variable_name="event_go_button",
        expected_value=True,
        comparison_operator=operator.eq,
        clearing_policy=py_trees.common.ClearingPolicy.ON_INITIALISE
    )
    teleport = Teleport(
        name=name,
        goal=gopher_navi_msgs.TeleportGoal(location="homebase", special_effects=True)
    )
    teleop_to_homebase.blackbox_level = py_trees.common.BlackBoxLevel.DETAIL
    teleop_and_teleport.add_child(teleop_to_homebase)
    teleop_to_homebase.add_child(flash_i_need_help)
    teleop_to_homebase.add_child(wait_for_go_button_press)
    teleop_and_teleport.add_child(teleport)
    return teleop_and_teleport


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

    behaviour = py_trees.subscribers.ToBlackboard(
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

    behaviour = py_trees.subscribers.ToBlackboard(
        name,
        topic_name=gopher.topics.pose,
        topic_type=geometry_msgs.PoseWithCovarianceStamped,
        blackboard_variables=blackboard_variables
    )
    return behaviour

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
        self.sent_goal = False

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
        self.sent_goal = True

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
        if self.action_client is not None and self.sent_goal:
            motion_state = self.action_client.get_state()
            if ((motion_state == actionlib_msgs.GoalStatus.PENDING) or (motion_state == actionlib_msgs.GoalStatus.ACTIVE) or
               (motion_state == actionlib_msgs.GoalStatus.PREEMPTING) or (motion_state == actionlib_msgs.GoalStatus.RECALLING)):
                self.action_client.cancel_goal()
        self.sent_goal = False

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
        self.action_client = None
        self.sent_goal = False
        self.goal = goal

    def initialise(self):
        self.logger.debug("  %s [Teleport::initialise()]" % self.name)
        self.sent_goal = False

    def setup(self, timeout):
        rospy.on_shutdown(functools.partial(self.stop, py_trees.Status.FAILURE))
        self.logger.debug("  %s [Teleport::setup()]" % self.name)
        if not self.action_client:
            self.gopher = gopher_configuration.Configuration(fallback_to_defaults=True)
            self.action_client = actionlib.SimpleActionClient(
                self.gopher.actions.teleport,
                gopher_navi_msgs.TeleportAction
            )
            if timeout is not None:
                if not self.action_client.wait_for_server(rospy.Duration(timeout)):
                    # replace with a py_trees exception!
                    self.logger.error("  %s [Teleport::setup()] could not connect to the action server" % self.name)
                    rospy.logerr("Behaviour [%s" % self.name + "] could not connect to the action server [%s]" % self.__class__.__name__)
                    self.action_client = None
                    return False
            # else just assume it's working, maybe None should be handled like infinite blocking
        return True

    def update(self):
        self.logger.debug("  %s [Teleport::update()]" % self.name)
        if not self.action_client:
            self.feedback_message = "no action client, did you call setup() on your tree?"
            return py_trees.Status.FAILURE
        # pity there is no 'is_connected' api like there is for c++
        if not self.sent_goal:
            self.action_client.send_goal(self.goal)
            self.sent_goal = True
            self.feedback_message = "sent goal to the action server"
            return py_trees.Status.RUNNING
        if self.action_client.get_state() == actionlib_msgs.GoalStatus.ABORTED:
            self.feedback_message = "aborted"
            return py_trees.Status.FAILURE
        result = self.action_client.get_result()
        # self.action_client.wait_for_result(rospy.Duration(0.1))  # < 0.1 is moot here - the internal loop is 0.1
        if result:
            if result.value == gopher_navi_msgs.TeleportResult.SUCCESS:
                self.feedback_message = "success"
                return py_trees.Status.SUCCESS
            else:
                self.feedback_message = result.message
                return py_trees.Status.FAILURE
        else:
            self.feedback_message = "waiting for teleport to init pose and clear costmaps"
            return py_trees.Status.RUNNING

    def terminate(self, new_status):
        # if we have an action client and the current goal has not already
        # succeeded, send a message to cancel the goal for this action client.
        # if self.action_client is not None and self.action_client.get_state() != actionlib_msgs.GoalStatus.SUCCEEDED:
        self.logger.debug("  %s [Teleport.terminate()][%s->%s]" % (self.name, self.status, new_status))
        if self.action_client is not None and self.sent_goal:
            motion_state = self.action_client.get_state()
            if ((motion_state == actionlib_msgs.GoalStatus.PENDING) or (motion_state == actionlib_msgs.GoalStatus.ACTIVE) or
               (motion_state == actionlib_msgs.GoalStatus.PREEMPTING) or (motion_state == actionlib_msgs.GoalStatus.RECALLING)):
                self.action_client.cancel_goal()
        self.sent_goal = False

##############################################################################
# Goal Finishing Behaviour
##############################################################################


class GoalFinishing(py_trees.Behaviour):
    """
    Simple goal finishing behaviour, just connects to the goal finishing action
    and runs that, nothing else.

    Blackboard Variables:

     - odom (r) [geometry_msgs/Pose] : current pose_base_rel_odom (from /gopher/odom)
     - pose (r) [geometry_msgs/Pose] : current pose_base_rel_map (from /navi/pose)
    """
    def __init__(self, name, goal_pose, distance_threshold=0.1, timeout=50.0):
        """
        The user should prepare the goal as there are quite a few ways that the
        goal message can be configured (see the comments in the msg file or
        just the help descriptions for the ``gopher_teleport` command line
        program for more information).

        :param geometry_msgs/Pose2D goal_pose: the goal pose (relative to the map frame) the robot shall finish at
        """
        super(GoalFinishing, self).__init__(name)
        self.action_client = None
        self.sent_goal = False
        self.goal = None
        self.goal_pose = goal_pose
        self.connected = False

        # parameters for re-trying
        self._distance_threshold = distance_threshold
        self._timeout = rospy.Duration(timeout)
        self._approach_timeout = rospy.Duration(0.7 * timeout)
        self._timeout = rospy.Duration(timeout)
        self._time_finishing_start = None
        self.blackboard = py_trees.blackboard.Blackboard()

    def setup(self, timeout):
        """
        Ros specific configuration
        """
        self.config = gopher_configuration.Configuration()
        self.action_client = actionlib.SimpleActionClient(
            self.config.actions.goal_finishing,
            gopher_navi_msgs.GoalFinishingAction
        )
        # should not have to wait as this will occur way after the teleport server is up
        self.connected = self.action_client.wait_for_server(rospy.Duration(timeout))
        if not self.connected:
            rospy.logwarn("GoalFinishing : behaviour could not connect with the goal finisher server.")
            self.action_client = None
            return False
        return True

    def initialise(self):
        self.logger.debug("  %s [GoalFinishing::initialise()]" % self.name)
        self.sent_goal = False

        # prepare the goal
        self.goal = gopher_navi_msgs.GoalFinishingGoal()
        self.goal.pose = geometry_msgs.PoseStamped()

        pose_goal_rel_map = geometry_msgs.Pose()
        pose_goal_rel_map.position.x = self.goal_pose.x
        pose_goal_rel_map.position.y = self.goal_pose.y
        pose_goal_rel_map.position.z = 0.0
        quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, self.goal_pose.theta)
        pose_goal_rel_map.orientation.x = quaternion[0]
        pose_goal_rel_map.orientation.y = quaternion[1]
        pose_goal_rel_map.orientation.z = quaternion[2]
        pose_goal_rel_map.orientation.w = quaternion[3]

        # convert goal pose from "map" into the "odom" frame, since this one is more reliable for short-distances
        pose_base_rel_odom = self.blackboard.odom.pose.pose
        pose_base_rel_map = self.blackboard.pose.pose.pose
        pose_map_rel_base = transform_utilities.get_inverse_pose(pose_base_rel_map)
        pose_map_rel_odom = transform_utilities.concatenate_poses(pose_map_rel_base, pose_base_rel_odom)
        pose_goal_rel_odom = transform_utilities.concatenate_poses(pose_goal_rel_map, pose_map_rel_odom)
        self.goal.pose.pose = pose_goal_rel_odom
        self.goal.pose.header.stamp = rospy.get_rostime()
        self.goal.pose.header.frame_id = self.blackboard.odom.header.frame_id
        self.goal.align_on_failure = False

        if not self.connected:
            rospy.logwarn("GoalFinishing : behaviour could not connect with the goal finisher server.")
            # we catch the failure in the first update() call
        else:
            self.action_client.send_goal(self.goal)
            self.sent_goal = True

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
                        if not self.goal.align_on_failure:
                            self.feedback_message = "failed to approach goal, but will try to align at least."
                            rospy.logwarn("GoalFinishing: %s" % self.feedback_message)
                            self.goal.align_on_failure = True
                    elif (current_finishing_duration > self._timeout):
                        self.feedback_message = "failed to approach and align with goal in time, but moving on."
                        rospy.logwarn("GoalFinishing: %s" % self.feedback_message)
                        return py_trees.Status.SUCCESS
                    else:
                        self.goal.align_on_failure = False
                    # re-try
                    self.action_client.send_goal(self.goal)
                    self.feedback_message = "waiting for the goal finisher to finish us"
                    return py_trees.Status.RUNNING
                else:
                    if (current_finishing_duration > self._timeout):
                        self.feedback_message = "approached, but failed to align with goal in time, but moving on."
                        rospy.logwarn("GoalFinishing: %s" % self.feedback_message)
                        return py_trees.Status.SUCCESS
                    else:
                        # try to align
                        self.goal.align_on_failure = True
                        self.action_client.send_goal(self.goal)
                        self.feedback_message = "approached, now aligning"
                        return py_trees.Status.RUNNING
        else:
            if (current_finishing_duration > self._approach_timeout) and\
               (current_finishing_duration <= self._timeout) and not self.goal.align_on_failure:
                # send a new goal, what will preempt the last one
                self.feedback_message = "failed to approach, switch priority to aligning with the goal"
                rospy.logwarn("GoalFinishing: %s" % self.feedback_message)
                self.goal.align_on_failure = True
                self.action_client.send_goal(self.goal)
                return py_trees.Status.RUNNING
            elif (current_finishing_duration > self._timeout):
                self.feedback_message = "time's up for aligning with the goal, moving on"
                rospy.logwarn("GoalFinishing: %s" % self.feedback_message)
                self.action_client.cancel_goal()
                return py_trees.Status.SUCCESS
            else:
                self.feedback_message = "waiting for goal finisher to finish us"
                return py_trees.Status.RUNNING

    def terminate(self, new_status):
        # if we have an action client and the current goal has not already
        # succeeded, send a message to cancel the goal for this action client.
        if self.action_client is not None and self.sent_goal:
            motion_state = self.action_client.get_state()
            if ((motion_state == actionlib_msgs.GoalStatus.PENDING) or (motion_state == actionlib_msgs.GoalStatus.ACTIVE) or
               (motion_state == actionlib_msgs.GoalStatus.PREEMPTING) or (motion_state == actionlib_msgs.GoalStatus.RECALLING)):
                self.action_client.cancel_goal()
        self.sent_goal = False

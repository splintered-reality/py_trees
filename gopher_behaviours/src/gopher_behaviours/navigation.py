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
import py_trees
import rospy
import tf

from . import interactions

##############################################################################
# Behaviour Factories
##############################################################################


def create_map_pose_to_blackboard_behaviour(
    name="PoseToBlackboard",
    blackboard_variable_name="pose"
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
        blackboard_variable_name=blackboard_variable_name
    )
    return behaviour

##############################################################################
# Behaviours
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
                 fail_if_complete=False
                 ):
        """
        :param str name: behaviour name
        :param str motion_type: rotation or translation (from gopher_std_msgs.SimpleMotionGoal, MOTION_ROTATE or MOTION_TRANSLATE)
        :param double motion_amount: how far the rotation (radians) or translation (m) should be
        :param bool unsafe: flag if you want the motion to be unsafe, i.e. not use the sensors
        :param bool keep_going: flag if you want the motion to return success in case the action aborts
        :param bool fail_if_complete: flag if you want the motion to return failure in case the motion is completed

        The ``keep_going`` flag is useful if you are attempting to rotate the robot out of harms way, but don't mind
        if it doesn't make the full specified rotation - this is oft used in navigation recovery style behaviours.

        The ``fail_if_complete`` flag is useful if you are expecting something to happen before the rotation end, i.e.
        if it gets to the end, it should be considered a failure - this could be used in a scanning rotation where it
        is looking for a landmark in the environment.
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
        if not self.action_client:
            self.gopher = gopher_configuration.Configuration()
            self.action_client = actionlib.SimpleActionClient(
                self.gopher.actions.simple_motion_controller,
                gopher_std_msgs.SimpleMotionAction
            )
            if timeout is not None:
                if not self.action_client.wait_for_server(rospy.Duration(timeout)):
                    rospy.logerr("Behaviour [%s" % self.name + "] could not connect to the simple motions action server [%s]" % self.__class__.__name__)
                    self.action_client = None
                    return False
            # else just assume it's working, maybe someone called this prior to ticking the behaviour
        return True

    def initialise(self):
        self.logger.debug("  %s [SimpleMotion::initialise()]" % self.name)
        if not self.action_client:
            self.setup(timeout=None)
        self.sent_goal = False

    def update(self):
        self.logger.debug("  %s [SimpleMotion::update()]" % self.name)
        if not self.sent_goal:
            # pity there is no 'is_connected' api
            if self.action_client.wait_for_server(rospy.Duration(0.01)):
                self.action_client.send_goal(self.goal)
                self.sent_goal = True
            else:
                self.feedback_message = "waiting for the simple motion server (correctly wired?)"
                return py_trees.Status.RUNNING
        if self.action_client is None:
            self.feedback_message = "action client wasn't connected"
            return py_trees.Status.FAILURE
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

    def stop(self, new_status=py_trees.Status.INVALID):
        # if we have an action client and the current goal has not already
        # succeeded, send a message to cancel the goal for this action client.
        # if self.action_client is not None and self.action_client.get_state() != actionlib_msgs.GoalStatus.SUCCEEDED:
        if self.action_client is None:
            return
        motion_state = self.action_client.get_state()
        if (motion_state == actionlib_msgs.GoalStatus.PENDING) or (motion_state == actionlib_msgs.GoalStatus.ACTIVE):
            self.action_client.cancel_goal()


##############################################################################
# Behaviours
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

    def stop(self, new_status=py_trees.Status.INVALID):
        """
        If we have an action client and the current goal has not already
        succeeded, send a message to cancel the goal for this action client.
        """
        if self.action_client is None:
            return
        motion_state = self.action_client.get_state()
        if (motion_state == actionlib_msgs.GoalStatus.PENDING) or (motion_state == actionlib_msgs.GoalStatus.ACTIVE):
            self.action_client.cancel_goal()


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


class GoalFinishing(py_trees.Behaviour):
    """
    Simple goal finishing behaviour; just connects to the goal finishing action and runs that, nothing else.

    :param geometry_msgs/Pose2D goal_pose  the goal pose the robot shall finish at
    """
    def __init__(self, name, goal_pose, distance_threshold=0.1, timeout=30.0):
        """
        The user should prepare the goal as there are quite a few ways that the
        goal message can be configured (see the comments in the msg file or
        just the help descriptions for the ``gopher_teleport` command line
        program for more information).

        :param gopher_navi_msgs.TeleportGoal goal: a suitably configured goal message.
        """
        super(GoalFinishing, self).__init__(name)
        self.goal = gopher_navi_msgs.GoalFinishingGoal()
        pose_stamped = geometry_msgs.PoseStamped()
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.frame_id = "map"
        pose_stamped.pose.position.x = goal_pose.x
        pose_stamped.pose.position.y = goal_pose.y
        pose_stamped.pose.position.z = 0.0
        quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, goal_pose.theta)
        pose_stamped.pose.orientation.x = quaternion[0]
        pose_stamped.pose.orientation.y = quaternion[1]
        pose_stamped.pose.orientation.z = quaternion[2]
        pose_stamped.pose.orientation.w = quaternion[3]
        self.goal.pose = pose_stamped
        self.goal.align_on_failure = False
        self.action_client = None
        self.gopher = gopher_configuration.Configuration()

        # parameters for re-trying
        self._distance_threshold = distance_threshold
        self._timeout = rospy.Duration(timeout)

        self._time_finishing_start = None
        self._final_goal_sent = False

    def initialise(self):
        self.logger.debug("  %s [GoalFinishing::initialise()]" % self.name)
        self.action_client = actionlib.SimpleActionClient(self.gopher.actions.goal_finishing,
                                                          gopher_navi_msgs.GoalFinishingAction)
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

        result = self.action_client.get_result()
        if result is not None:
            if result.value == gopher_navi_msgs.GoalFinishingResult.SUCCESS:
                self.feedback_message = "success"
                return py_trees.Status.SUCCESS
            else:
                if (result.goal_distance > self._distance_threshold) and not self._final_goal_sent:
                    if (rospy.Time.now() - self._time_finishing_start) > self._timeout:
                        # at last just try to align
                        self.goal.align_on_failure = True
                        if not self._final_goal_sent:
                            self._final_goal_sent = True
                    else:
                        self.goal.align_on_failure = False
                    # re-try
                    self.action_client.send_goal(self.goal)
                    return py_trees.Status.RUNNING
                else:
                    # consider success
                    rospy.logwarn("GoalFinishing : Failed to finish at the goal, but considering finished anyway.")
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


class PoseIntialisation(py_trees.Sequence):
    """
    This class implements the sequence of initialising the robot's pose. The sequence consists of the following:

    - enable AR (behaviour)
    - initialise (selector)
        - check dslam init status (behaviour)
        - manual init (sequence)
            - check cancel button (behaviour)
            - wait for confirm button press (behaviour)
            - teleport to home base (behaviour)
        - rotate (behaviour)
    - disable AR (behaviour)
    """
    def __init__(self, name):
        """
        Put together the pose intialisation sequence
        """
        super(PoseIntialisation, self).__init__(name)
        self._config = gopher_configuration.Configuration()

        # enable tracker
        self.add_child(interactions.ControlARMarkerTracker("Enable AR Marker Tracker",
                                                           self._config.topics.ar_tracker_long_range,
                                                           True))
        initialise = py_trees.Selector("Initialise")
        # check status
        check_status = py_trees.Sequence("Verify Localisation")
        check_status.add_child(py_trees.CheckSubscriberVariable(
            name="Check ELF localiser state",
            topic_name=self._config.topics.elf_status,
            topic_type=elf_msgs.ElfLocaliserStatus,
            variable_name="status",
            expected_value=elf_msgs.ElfLocaliserStatus.STATUS_WORKING,
            fail_if_no_data=True,
            fail_if_bad_comparison=True,
            monitor_continuously=True)
        )
        notify_done = interactions.SendNotification(
            "Intialised",
            led_pattern=gopher_std_msgs.LEDStrip.AROUND_RIGHT_GREEN,
            sound=self._config.sounds.done,
            message="intialised pose"
        )
        notify_done.add_child(py_trees.Pause("Celebrate the Success", 2.0))
        check_status.add_child(notify_done)
        initialise.add_child(check_status)
        # manual init
        manual_init = py_trees.Sequence("Manual Initialisation")
        manual_init.add_child(interactions.CheckButtonPressed("Check Cancel Button", self._config.buttons.stop))
        manual_init.add_child(interactions.WaitForButton("Wait for Go Button Trigger", self._config.buttons.go))
        manual_init.add_child(Teleport("Teleport to Homebase",
                                       gopher_navi_msgs.TeleportGoal(location="homebase", special_effects=True)))
        manual_init.add_child(time.Timeout("Wait for Initialisation", 600.0))
        initialise.add_child(manual_init)
        # auto init
        auto_init = py_trees.Selector("Autonomous Initialisation")
        rotate = SimpleMotion(name="Rotate", motion_amount=(2 * math.pi), keep_going=False, fail_if_complete=True)
        auto_init.add_child(rotate)
        wait_for_retry = py_trees.Pause("Wait for Retry", 0.0)
        auto_init.add_child(wait_for_retry)
        initialise.add_child(auto_init)
        # timeout
        initialise.add_child(py_trees.Timeout("Initialisation Timeout", 600.0))
        self.add_child(initialise)
        # disable tracker
        self.add_child(interactions.ControlARMarkerTracker("Disable AR Marker Tracker",
                                                           self._config.topics.ar_tracker_long_range,
                                                           False))

    def initialise(self):
        """
        Initialise the pose intialisation sequence
        """
        super(PoseIntialisation, self).initialise()

    def stop(self, new_status=py_trees.Status.INVALID):
        """
        Stop the pose intialisation sequence
        """
        super(PoseIntialisation, self).stop(new_status)

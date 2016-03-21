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
import geometry_msgs.msg as geometry_msgs
import gopher_configuration
import gopher_navi_msgs.msg as gopher_navi_msgs
import move_base_msgs.msg as move_base_msgs
import gopher_std_msgs.msg as gopher_std_msgs
import gopher_std_msgs.srv as gopher_std_srvs
import py_trees
import rospy
import tf


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
        topic_name=gopher.topics.odom,
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

    .. note:: This class assumes a higher level pastafarian has ensured the action client is available.
    """
    def __init__(self, name="simple_motions",
                 motion_type=gopher_std_msgs.SimpleMotionGoal.MOTION_ROTATE,
                 motion_amount=0,
                 unsafe=False
                 ):
        """
        :param str name: behaviour name
        :param str motion_type: rotation or translation (from gopher_std_msgs.SimpleMotionGoal, MOTION_ROTATE or MOTION_TRANSLATE)
        :param double motion_amount: how far the rotation (radians) or translation (m) should be
        :param bool unsafe: flag if you want the motion to be unsafe, i.e. not use the sensors
        """
        super(SimpleMotion, self).__init__(name)
        self.gopher = None
        self.action_client = None
        self.has_goal = False
        self.goal = gopher_std_msgs.SimpleMotionGoal()
        self.goal.motion_type = motion_type
        self.goal.motion_amount = motion_amount
        self.goal.unsafe = unsafe

    def setup_ros(self, timeout):
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
            if not self.action_client.wait_for_server(rospy.Duration(timeout)):
                rospy.logerr("Behaviour [%s" % self.name + "] could not connect to the simple motions action server [%s]" % self.__class__.__name__)
                self.action_client = None
                return False
        return True

    def initialise(self):
        self.logger.debug("  %s [SimpleMotion::initialise()]" % self.name)
        if not self.action_client:
            if not self.setup_ros(timeout=0.01):
                return
        self.action_client.send_goal(self.goal)

    def update(self):
        self.logger.debug("  %s [SimpleMotion::update()]" % self.name)
        if self.action_client is None:
            self.feedback_message = "action client wasn't connected"
            return py_trees.Status.FAILURE
        if self.action_client.get_state() == actionlib_msgs.GoalStatus.ABORTED:
            self.feedback_message = "simple motion aborted, but we keep on marching forward"
            return py_trees.Status.SUCCESS
        result = self.action_client.get_result()
        if result:
            self.feedback_message = "goal reached"
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

    def setup_ros(self, timeout):
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
            if not self.setup_ros(timeout=0.01):
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

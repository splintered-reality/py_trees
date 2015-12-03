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
import gopher_configuration
import gopher_navi_msgs.msg as gopher_navi_msgs
import move_base_msgs.msg as move_base_msgs
import gopher_std_msgs.msg as gopher_std_msgs
import gopher_std_msgs.srv as gopher_std_srvs
import py_trees
import rospy
import tf

##############################################################################
# Classes
##############################################################################


class SimpleMotion():
    def __init__(self):
        self.config = gopher_configuration.Configuration()
        self.action_client = actionlib.SimpleActionClient(self.config.actions.simple_motion_controller,
                                                          gopher_std_msgs.SimpleMotionAction)
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
        # TODO : when marco has fixed the psd's, this needs to become False
        goal.unsafe = True
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

    def stop(self):
        if self.has_goal:
            motion_state = self.action_client.get_state()
            if (motion_state == actionlib_msgs.GoalStatus.PENDING or motion_state == actionlib_msgs.GoalStatus.ACTIVE):
                self.action_client.cancel_goal()

##############################################################################
# Behaviours
##############################################################################


class MoveIt(py_trees.Behaviour):
    """
    Simplest kind of move it possible. Just connects to the move base action
    and runs that, nothing else.

    :param bool dont_ever_give_up: temporary variable. Retry if move base action fails.

    Dont ever give up is our default action for moving around until we decide otherwise.
    Set it true here, and later we can disable it here and set it for the few specific
    instances that require it. That will save us from having to do an exhaustive search
    and destroy later.
    """
    def __init__(self, name, pose, dont_ever_give_up=True):
        super(MoveIt, self).__init__(name)
        self.pose = pose
        self.action_client = None
        self.gopher = gopher_configuration.Configuration()
        self.dont_ever_give_up = dont_ever_give_up
        rospy.wait_for_service(self.gopher.services.notification)
        self.notify_srv = rospy.ServiceProxy(self.gopher.services.notification, gopher_std_srvs.Notify)
        self.goal = None

    def initialise(self):
        self.logger.debug("  %s [MoveIt::initialise()]" % self.name)
        self.action_client = actionlib.SimpleActionClient(self.gopher.actions.move_base, move_base_msgs.MoveBaseAction)

        connected = self.action_client.wait_for_server(rospy.Duration(0.5))
        if not connected:
            rospy.logwarn("MoveIt : could not connect to move base at {0}.".format(self.gopher.actions.move_base))
            self.action_client = None
        else:
            self.goal = move_base_msgs.MoveBaseGoal()  # don't yet care about the target
            self.goal.target_pose.header.frame_id = "map"
            self.goal.target_pose.pose.position.x = self.pose.x
            self.goal.target_pose.pose.position.y = self.pose.y
            quaternion = tf.transformations.quaternion_from_euler(0, 0, self.pose.theta)
            self.goal.target_pose.pose.orientation.x = quaternion[0]
            self.goal.target_pose.pose.orientation.y = quaternion[1]
            self.goal.target_pose.pose.orientation.z = quaternion[2]
            self.goal.target_pose.pose.orientation.w = quaternion[3]
            self.action_client.send_goal(self.goal)

    def update(self):
        self.logger.debug("  %s [MoveIt::update()]" % self.name)
        if self.action_client is None:
            self.feedback_message = "action client couldn't connect"
            return py_trees.Status.INVALID
        if self.action_client.get_state() == actionlib_msgs.GoalStatus.ABORTED:
            if self.dont_ever_give_up:
                self.feedback_message = "move base aborted, but we dont ever give up...tallyho!"
                # this will light up for the default period of the notifier (typically 10s)
                self.publisher.publish(
                    gopher_std_msgs.Notification(
                        led_pattern=self.gopher.led_patterns.dab_dab_hae,
                        message=self.feedback_message
                    )
                )
                self.action_client.send_goal(self.goal)
                return py_trees.Status.RUNNING
            else:
                self.feedback_message = "move base aborted"
                return py_trees.Status.FAILURE

        result = self.action_client.get_result()
        # self.action_client.wait_for_result(rospy.Duration(0.1))  # < 0.1 is moot here - the internal loop is 0.1
        if result:
            self.feedback_message = "goal reached"
            return py_trees.Status.SUCCESS
        else:
            self.feedback_message = "moving"
            return py_trees.Status.RUNNING

    def stop(self, new_status):
        # if we have an action client and the current goal has not already
        # succeeded, send a message to cancel the goal for this action client.
        if self.action_client is not None and self.action_client.get_state() != actionlib_msgs.GoalStatus.SUCCEEDED:
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

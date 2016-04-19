#!/usr/bin/env python
#
# License: Yujin
#
##############################################################################
# Documentation
##############################################################################

"""
.. module:: simple_motions
   :platform: Unix
   :synopsis: Behaviours working with the simple motions controller.

Oh my spaghettified magnificence,
Bless my noggin with a tickle from your noodly appendages!
"""

##############################################################################
# Imports
##############################################################################

import actionlib
import actionlib_msgs.msg as actionlib_msgs
import gopher_configuration
import gopher_std_msgs.msg as gopher_std_msgs
import math
import py_trees
import rospy

##############################################################################
# Helpers
##############################################################################


def create_rotate_ad_nauseum(timeout):
    """
    Create a behaviour that continually tries to rotate, despite failures
    until a timeout is triggered.

    Blackbox Level: DETAIL

    :param float timeout:
    :returns: subtree
    """
    rotate_ad_nauseum = py_trees.composites.Parallel(
        name="Rotate Ad Nauseum",
        policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL
    )
    rotate = py_trees.meta.success_is_running(SimpleMotion)(name="Rotate", motion_amount=(0.25 * math.pi))
    timeout = py_trees.meta.inverter(py_trees.timers.Timer)(name="Timeout", duration=15.0)
    rotate_ad_nauseum.add_child(rotate)
    rotate_ad_nauseum.add_child(timeout)
    rotate_ad_nauseum.blackbox_level = py_trees.common.BlackBoxLevel.DETAIL
    return rotate_ad_nauseum


##############################################################################
# Classes
##############################################################################


class SimpleMotion(py_trees.Behaviour):
    """
    Interface to the simple motions controller.
    """
    def __init__(self, name="simple_motions",
                 motion_type=gopher_std_msgs.SimpleMotionGoal.MOTION_ROTATE,
                 motion_amount=0,
                 unsafe=False,
                 keep_trying_timeout=0.0
                 ):
        """
        :param str name: behaviour name
        :param str motion_type: rotation or translation (from gopher_std_msgs.SimpleMotionGoal, MOTION_ROTATE or MOTION_TRANSLATE)
        :param double motion_amount: how far the rotation (radians) or translation (m) should be
        :param bool unsafe: flag if you want the motion to be unsafe, i.e. not use the sensors
        :param bool keep_trying_timeout: keep trying rather than aborting (up to this timeout) if it detects obstacles

        The ``keep_trying_timeout`` is passed to the simple motion server. It is not a total timeout for the
        simple motion to take place, instead it is the duration between detecting an obstacle and the time it will give
        up. A value of 0.0 will give up immediately. A negative value will never give up.
        """
        super(SimpleMotion, self).__init__(name)
        self.gopher = None
        self.action_client = None
        self.sent_goal = False
        self.goal = gopher_std_msgs.SimpleMotionGoal()
        self.goal.motion_type = motion_type
        self.goal.motion_amount = motion_amount
        self.goal.unsafe = unsafe
        self.goal.keep_trying_timeout = keep_trying_timeout

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
        self.logger.debug("  %s [SimpleMotion::setup()]" % self.name)
        self.gopher = gopher_configuration.Configuration()
        self.action_client = actionlib.SimpleActionClient(
            self.gopher.actions.simple_motion_controller,
            gopher_std_msgs.SimpleMotionAction
        )
        if not self.action_client.wait_for_server(rospy.Duration(timeout)):
            # replace with a py_trees exception!
            self.logger.error("  %s [SimpleMotion::setup()] could not connect to the docking action server" % self.name)
            rospy.logerr("Behaviour [%s" % self.name + "] could not connect to the simple motions action server [%s]" % self.__class__.__name__)
            self.action_client = None
            return False
        return True

    def initialise(self):
        self.logger.debug("  %s [SimpleMotion::initialise()]" % self.name)
        self.sent_goal = False

    def update(self):
        self.logger.debug("  %s [SimpleMotion::update()]" % self.name)
        if not self.action_client:
            self.feedback_message = "no action client, did you call setup() on your tree?"
            return py_trees.Status.INVALID
        # pity there is no 'is_connected' api like there is for c++
        if not self.sent_goal:
            self.action_client.send_goal(self.goal)
            self.sent_goal = True
            self.feedback_message = "sent goal to the action server"
            return py_trees.Status.RUNNING
        if self.action_client.get_state() == actionlib_msgs.GoalStatus.ABORTED:
            print("Aborted")
            self.feedback_message = "simple motion aborted"
            return py_trees.Status.FAILURE
        result = self.action_client.get_result()
        if result:
            self.feedback_message = "goal reached"
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
            if ((motion_state == actionlib_msgs.GoalStatus.PENDING) or (motion_state == actionlib_msgs.GoalStatus.ACTIVE) or
               (motion_state == actionlib_msgs.GoalStatus.PREEMPTING) or (motion_state == actionlib_msgs.GoalStatus.RECALLING)):
                self.action_client.cancel_goal()
        self.sent_goal = False

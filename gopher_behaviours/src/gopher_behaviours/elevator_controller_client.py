#
# License: Yujin
#
##############################################################################
# Description
##############################################################################

"""
.. module:: elevator_controller_client
   :platform: Unix
   :synopsis: Elevator controller related behaviours

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
import gopher_std_msgs.msg as gopher_std_msgs
import py_trees
import rospy

##############################################################################
# Helpers
##############################################################################


##############################################################################
# Behaviours
##############################################################################

class ElevatorTransfer(py_trees.Behaviour):
    """
    Interface to the elevator controller.
    """
    def __init__(self, name="elevator_transfer", boarding=True):
        """
        :param str name: behaviour name
        :param bool boarding: set to true for entering the elevator, false for exiting
        """
        super(ElevatorTransfer, self).__init__(name)
        self.gopher = None
        self.action_client = None
        self.sent_goal = False
        self.goal = gopher_std_msgs.ElevatorTransferGoal()
        self.goal.enter = boarding
        self.action_feedback = None

    def setup(self, timeout):
        """
        Wait for the action server to come up.

        :param double timeout: time to wait (0.0 is blocking forever)
        :returns: whether it timed out waiting for the server or not.
        :rtype: boolean
        """
        self.logger.debug("  %s [ElevatorTransfer::setup()]" % self.name)
        self.gopher = gopher_configuration.Configuration()
        self.action_client = actionlib.SimpleActionClient(
            self.gopher.actions.elevator_transfer,
            gopher_std_msgs.ElevatorTransferAction
        )
        if not self.action_client.wait_for_server(rospy.Duration(timeout)):
            # replace with a py_trees exception!s
            self.logger.error("  %s [ElevatorTransfer::setup()] could not connect to the elevator transfer server"
                              % self.name)
            rospy.logerr("Behaviour [%s" % self.name + "] could not connect to the elevator transfer action server [%s]"
                         % self.__class__.__name__)
            self.action_client = None
            return False
        return True

    def _feedbackCB(self, goal_feedback):
        self.action_feedback = goal_feedback.status

    def initialise(self):
        self.logger.debug("  %s [ElevatorTransfer::initialise()]" % self.name)
        self.sent_goal = False

    def update(self):
        self.logger.debug("  %s [ElevatorTransfer::update()]" % self.name)
        if not self.action_client:
            self.feedback_message = "no action client, did you call setup() on your tree?"
            return py_trees.Status.INVALID
        # pity there is no 'is_connected' api like there is for c++
        if not self.sent_goal:
            self.action_client.send_goal(self.goal, None, None, self._feedbackCB)
            self.sent_goal = True
            self.feedback_message = "sent goal to the action server"
            return py_trees.Status.RUNNING
        if self.action_client.get_state() == actionlib_msgs.GoalStatus.ABORTED:
            result = self.action_client.get_result()
            self.feedback_message = result.message
            return py_trees.Status.FAILURE
        result = self.action_client.get_result()
        if result:
            self.feedback_message = "transfered"
            return py_trees.Status.SUCCESS
        else:
            if self.action_feedback:
                self.feedback_message = "transferring (" + self.action_feedback + ")"
                self.action_feedback = None
            else:
                self.feedback_message = "transferring"
            return py_trees.Status.RUNNING

    def terminate(self, new_status):
        # if we have an action client and the current goal has not already
        # succeeded, send a message to cancel the goal for this action client.
        # if self.action_client is not None and self.action_client.get_state() != actionlib_msgs.GoalStatus.SUCCEEDED:
        self.logger.debug("  %s [ElevatorTransfer.terminate()][%s->%s]" % (self.name, self.status, new_status))
        if self.action_client is not None and self.sent_goal:
            motion_state = self.action_client.get_state()
            if ((motion_state == actionlib_msgs.GoalStatus.PENDING) or
               (motion_state == actionlib_msgs.GoalStatus.ACTIVE) or
               (motion_state == actionlib_msgs.GoalStatus.PREEMPTING) or
               (motion_state == actionlib_msgs.GoalStatus.RECALLING)):
                self.action_client.cancel_goal()
        self.sent_goal = False

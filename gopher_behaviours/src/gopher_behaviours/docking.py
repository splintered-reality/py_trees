#!/usr/bin/env python
import rospy
import py_trees
import actionlib
import gopher_configuration
import functools
from py_trees.blackboard import Blackboard
import gopher_std_msgs.msg as gopher_std_msgs
import actionlib_msgs.msg as actionlib_msgs


class AutoDock(py_trees.Behaviour):
    """
    Behaviour interface to the docking/undocking controller.

    Blackboard Variables:

     - undocked (w) [bool]: if undocking, true if the last update was a success, false otherwise (unused for docking)
    """
    def __init__(self, name="AutoDock", undock=False):
        """
        :param str name: the behaviour name
        :param bool undock: do undocking instead of docking.
        """
        super(AutoDock, self).__init__(name)
        rospy.on_shutdown(functools.partial(self.stop, py_trees.Status.FAILURE))
        self.action_client = None
        self.sent_goal = False
        self.blackboard = Blackboard()
        self.goal = gopher_std_msgs.AutonomousDockingGoal()
        self.goal.command = gopher_std_msgs.AutonomousDockingGoal.DOCK if not undock else gopher_std_msgs.AutonomousDockingGoal.UNDOCK

    def setup(self, timeout):
        self.logger.debug("  %s [AutoDock::setup()]" % self.name)
        if not self.action_client:
            self.gopher = gopher_configuration.Configuration()
            self.action_client = actionlib.SimpleActionClient(
                self.gopher.actions.autonomous_docking,
                gopher_std_msgs.AutonomousDockingAction)
            if timeout is not None:
                if not self.action_client.wait_for_server(rospy.Duration(timeout)):
                    # replace with a py_trees exception!
                    self.logger.error("  %s [AutoDock::setup()] could not connect to the docking action server" % self.name)
                    rospy.logerr("Behaviour [%s" % self.name + "] could not connect to the docking action server [%s]" % self.__class__.__name__)
                    self.action_client = None
                    return False
            # else just assume it's working, maybe None should be handled like infinite blocking
        return True

    def initialise(self):
        self.logger.debug("  %s [AutoDock::initialise()]" % self.name)
        self.sent_goal = False
        if self.goal.command == gopher_std_msgs.AutonomousDockingGoal.UNDOCK:
            self.blackboard.undocked = False

    def update(self):
        self.logger.debug("  %s [AutoDock::update()]" % self.name)
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
        if result:
            if result.value == gopher_std_msgs.AutonomousDockingResult.SUCCESS:
                if self.goal.command == gopher_std_msgs.AutonomousDockingGoal.UNDOCK:
                    self.blackboard.undocked = True
                self.feedback_message = result.message
                return py_trees.Status.SUCCESS
            elif result.value == gopher_std_msgs.AutonomousDockingResult.ABORTED_OBSTACLES:
                self.feedback_message = result.message
                return py_trees.Status.FAILURE
            else:
                self.feedback_message = "got unknown result value from docking controller"
                return py_trees.Status.FAILURE
        else:
            self.feedback_message = "moving"
            return py_trees.Status.RUNNING

    def terminate(self, new_status):
        # if we have an action client and the current goal has not already
        # succeeded, send a message to cancel the goal for this action client.
        # if self.action_client is not None and self.action_client.get_state() != actionlib_msgs.GoalStatus.SUCCEEDED:
        self.logger.debug("  %s [AutoDock.terminate()][%s->%s]" % (self.name, self.status, new_status))
        if self.action_client is not None and self.sent_goal:
            motion_state = self.action_client.get_state()
            if (motion_state == actionlib_msgs.GoalStatus.PENDING) or (motion_state == actionlib_msgs.GoalStatus.ACTIVE):
                self.action_client.cancel_goal()
        self.sent_goal = False

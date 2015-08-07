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
import move_base_msgs.msg as move_base_msgs
import py_trees
import tf
import rospy

##############################################################################
# Class
##############################################################################


class Dock(py_trees.Behaviour):
    """
    Engage the docking behaviour
    """
    def __init__(self, name):
        super(Dock, self).__init__(name)

    def update(self):
        self.logger.debug("  %s [Dock::update()]" % self.name)
        return py_trees.Status.SUCCESS


class UnDock(py_trees.Behaviour):
    """
    Leave the docking bay. This is typically a specialised maneuvre so that
    the robot can begin localising.
    """
    def __init__(self, name):
        super(UnDock, self).__init__(name)

    def update(self):
        self.logger.debug("  %s [UnDock::update()]" % self.name)
        return py_trees.Status.SUCCESS


class MoveToGoal(py_trees.Behaviour):
    def __init__(self, name, pose):
        super(MoveToGoal, self).__init__(name)
        self.pose = pose
        self.action_client = None

    def initialise(self):
        self.logger.debug("  %s [MoveToGoal::initialise()]" % self.name)
        self.action_client = actionlib.SimpleActionClient('~move_base', move_base_msgs.MoveBaseAction)
        connected = self.action_client.wait_for_server(rospy.Duration(0.2))
        if not connected:
            rospy.logwarn("MoveToGoal : could not connect with move base.")
            self.action_client = None
        else:
            goal = move_base_msgs.MoveBaseGoal()  # don't yet care about the target
            goal.target_pose.pose.position.x = self.pose.x
            goal.target_pose.pose.position.y = self.pose.y
            quaternion = tf.transformations.quaternion_from_euler(0, 0, self.pose.theta)
            goal.target_pose.pose.orientation.x = quaternion[0]
            goal.target_pose.pose.orientation.y = quaternion[1]
            goal.target_pose.pose.orientation.z = quaternion[2]
            goal.target_pose.pose.orientation.w = quaternion[3]
            self.action_client.send_goal(goal)

    def update(self):
        self.logger.debug("  %s [MoveToGoal::update()]" % self.name)
        if self.action_client is None:
            self.feedback_message = "action client couldn't connect"
            return py_trees.Status.INVALID
        result = self.action_client.get_result()
        # self.action_client.wait_for_result(rospy.Duration(0.1))  # < 0.1 is moot here - the internal loop is 0.1
        if result:
            self.feedback_message = "goal reached"
            return py_trees.Status.SUCCESS
        else:
            self.feedback_message = "moving"
            return py_trees.Status.RUNNING

    def abort(self, new_status):
        if self.action_client is not None:
            self.action_client.cancel_all_goals()


class GoHome(py_trees.Behaviour):
    def __init__(self, name):
        super(GoHome, self).__init__(name)

    def update(self):
        self.logger.debug("  %s [GoHome::update()]" % self.name)
        return py_trees.Status.SUCCESS

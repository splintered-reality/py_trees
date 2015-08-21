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
from actionlib_msgs.msg import GoalStatus
import move_base_msgs.msg as move_base_msgs
import std_msgs.msg as std_msgs
import py_trees
import tf
import rospy
from .blackboard import Blackboard

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
        self.blackboard = Blackboard()
        
        self._honk_publisher = None
        try:
            if rospy.get_param("~enable_honks"):
                honk_topic = rospy.get_param("~moving_honk")
                self._honk_publisher = rospy.Publisher("/gopher/commands/sounds/" + honk_topic, std_msgs.Empty, queue_size=1)
        except KeyError:
            rospy.logwarn("Gopher Deliveries : Could not find param to initialise honks.")
            pass

    def initialise(self):
        self.logger.debug("  %s [MoveToGoal::initialise()]" % self.name)
        self.action_client = actionlib.SimpleActionClient('~move_base', move_base_msgs.MoveBaseAction)
        
        connected = self.action_client.wait_for_server(rospy.Duration(0.5))
        if not connected:
            rospy.logwarn("MoveToGoal : could not connect with move base.")
            self.action_client = None
        else:
            goal = move_base_msgs.MoveBaseGoal()  # don't yet care about the target
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.pose.position.x = self.pose.x
            goal.target_pose.pose.position.y = self.pose.y
            quaternion = tf.transformations.quaternion_from_euler(0, 0, self.pose.theta)
            goal.target_pose.pose.orientation.x = quaternion[0]
            goal.target_pose.pose.orientation.y = quaternion[1]
            goal.target_pose.pose.orientation.z = quaternion[2]
            goal.target_pose.pose.orientation.w = quaternion[3]
            self.action_client.send_goal(goal)

        if self._honk_publisher:
            self._honk_publisher.publish(std_msgs.Empty())

    def update(self):
        self.logger.debug("  %s [MoveToGoal::update()]" % self.name)
        if self.action_client is None:
            self.feedback_message = "action client couldn't connect"
            return py_trees.Status.INVALID
        result = self.action_client.get_result()
        # self.action_client.wait_for_result(rospy.Duration(0.1))  # < 0.1 is moot here - the internal loop is 0.1
        if result:
            self.feedback_message = "goal reached"
            self.blackboard.traversed_locations.append(self.blackboard.remaining_locations.pop(0))
            return py_trees.Status.SUCCESS
        else:
            self.feedback_message = "moving"
            return py_trees.Status.RUNNING

    def abort(self, new_status):
        # if we have an action client and the current goal has not already
        # succeeded, send a message to cancel the goal for this action client.
        if self.action_client is not None and self.action_client.get_state() != GoalStatus.SUCCEEDED:
            self.action_client.cancel_goal()

class GoHome(py_trees.Behaviour):
    def __init__(self, name):
        super(GoHome, self).__init__(name)

    def update(self):
        self.logger.debug("  %s [GoHome::update()]" % self.name)
        return py_trees.Status.SUCCESS

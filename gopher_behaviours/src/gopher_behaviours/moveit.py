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
import gopher_std_msgs.msg as gopher_std_msgs
import std_msgs.msg as std_msgs
import somanet_msgs.msg as somanet_msgs
import py_trees
import tf
import rospy
from .blackboard import Blackboard

##############################################################################
# Class
##############################################################################

class DockSelector(py_trees.Selector):

    def __init__(self, name):
        super(DockSelector, self).__init__(name, children=[Dock('Attempt Docking'), WaitForCharge('Wait for help')])

class Dock(py_trees.Behaviour):
    """
    Engage the docking behaviour
    """
    # if one dock object failed to dock, then all others will not do anything
    # until the waitforcharge behaviour successfully completes
    def __init__(self, name):
        super(Dock, self).__init__(name)
        self.action_client = actionlib.SimpleActionClient('autonomous_docking', gopher_std_msgs.AutonomousDockingAction)
        self._honk_publisher = None
        try:
            if rospy.get_param("~enable_honks") and rospy.get_param("~docking_honk"):
                honk_topic = rospy.get_param("~docking_honk")
                self._honk_publisher = rospy.Publisher("/gopher/commands/sounds/" + honk_topic, std_msgs.Empty, queue_size=1)
        except KeyError:
            rospy.logwarn("Gopher Deliveries : Could not find param to initialise honks.")
            pass

    def initialise(self):
        if self.status == py_trees.Status.FAILURE:
            return
            
        self.connected = self.action_client.wait_for_server(rospy.Duration(0.5))
        if not self.connected:
            rospy.logwarn("Dock : could not connect to autonomous docking server.")
        else:
            goal = gopher_std_msgs.AutonomousDockingGoal
            goal.command = gopher_std_msgs.AutonomousDockingGoal.DOCK
            self.action_client.send_goal(goal)

        if self._honk_publisher:
            self._honk_publisher.publish(std_msgs.Empty())

    def update(self):
        self.logger.debug("  %s [Dock::update()]" % self.name)
        if not self.connected:
            self.feedback_message = "Action client failed to connect"
            return py_trees.Status.INVALID

        result = self.action_client.get_result()
        if result:
            if result.value == gopher_std_msgs.AutonomousDockingResult.SUCCESS:
                self.feedback_message = result.message
                return py_trees.Status.SUCCESS
            elif result.value == gopher_std_msgs.AutonomousDockingResult.ABORTED_OBSTACLES:
                self.feedback_message = result.message
                Dock.failed = True
                return py_trees.Status.FAILURE
            else:
                self.feedback_message = "Got unknown result value from docking controller"
                return py_trees.Status.FAILURE
        else:
            self.feedback_message = "Docking in progress"
            return py_trees.Status.RUNNING

class WaitForCharge(py_trees.Behaviour):

    def __init__(self, name):
        super(WaitForCharge, self).__init__(name)
        self._notify_publisher = rospy.Publisher('~display_notification', gopher_std_msgs.Notification, queue_size=1)
        self._battery_subscriber = rospy.Subscriber("~battery", somanet_msgs.SmartBatteryStatus, self.battery_callback)
        self.notify_timer = None
        self.charge_state = somanet_msgs.SmartBatteryStatus.DISCHARGING
        
    def initialise(self):
        rospy.loginfo("init charge")
        self.send_notification(None)
        # Notifications last for some amount of time before LEDs go back to
        # displaying the battery status. Need to send message repeatedly until
        # the behaviour completes.
        self.notify_timer = rospy.Timer(rospy.Duration(5), self.send_notification)

    def send_notification(self, timer):
        self._notify_publisher.publish(gopher_std_msgs.Notification(led_pattern=gopher_std_msgs.Notification.SOLID_RED))

    def battery_callback(self, msg):
        self.charge_state = msg.charge_state

    def update(self):
        rospy.loginfo("update charge")
        if self.charge_state == somanet_msgs.SmartBatteryStatus.CHARGING:
            rospy.loginfo("charge done!")
            return py_trees.Status.SUCCESS
        else:
            rospy.loginfo("charge wating...")
            return py_trees.Status.RUNNING

    def abort(self, new_status):
        rospy.loginfo("charge aborted")
        if self.notify_timer:
            self.notify_timer.shutdown()

class Undock(py_trees.Behaviour):
    """
    Leave the docking bay. This is typically a specialised maneuvre so that
    the robot can begin localising.
    """
    def __init__(self, name):
        super(Undock, self).__init__(name)
        self.action_client = actionlib.SimpleActionClient('autonomous_docking', gopher_std_msgs.AutonomousDockingAction)
        self._honk_publisher = None
        try:
            if rospy.get_param("~enable_honks") and rospy.get_param("~undocking_honk"):
                honk_topic = rospy.get_param("~undocking_honk")
                self._honk_publisher = rospy.Publisher("/gopher/commands/sounds/" + honk_topic, std_msgs.Empty, queue_size=1)
        except KeyError:
            rospy.logwarn("Gopher Deliveries : Could not find param to initialise honks.")
            pass

    def initialise(self):
        self.connected = self.action_client.wait_for_server(rospy.Duration(0.5))
        if not self.connected:
            rospy.logwarn("Undock : could not connect to autonomous docking server.")
        else:
            goal = gopher_std_msgs.AutonomousDockingGoal
            goal.command = gopher_std_msgs.AutonomousDockingGoal.UNDOCK
            self.action_client.send_goal(goal)

        if self._honk_publisher:
            self._honk_publisher.publish(std_msgs.Empty())


    def update(self):
        self.logger.debug("  %s [Undock::update()]" % self.name)
        if not self.connected:
            self.feedback_message = "Action client failed to connect"
            return py_trees.Status.INVALID

        result = self.action_client.get_result()
        if result:
            if result.value == gopher_std_msgs.AutonomousDockingResult.SUCCESS:
                self.feedback_message = result.message
                return py_trees.Status.SUCCESS
            elif result.value == gopher_std_msgs.AutonomousDockingResult.ABORTED_OBSTACLES:
                self.feedback_message = result.message
                return py_trees.Status.FAILURE
            else:
                self.feedback_message = "Got unknown result value from undocking controller"
                return py_trees.Status.INVALID
        else:
            self.feedback_message = "Unocking in progress"
            return py_trees.Status.RUNNING

class MoveToGoal(py_trees.Behaviour):
    def __init__(self, name, pose):
        super(MoveToGoal, self).__init__(name)
        self.pose = pose
        self.action_client = None
        self.blackboard = Blackboard()
        
        self._honk_publisher = None
        try:
            if rospy.get_param("~enable_honks") and rospy.get_param("~moving_honk"):
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

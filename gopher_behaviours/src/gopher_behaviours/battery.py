#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/yujinrobot/gopher_crazy_hospital/license/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
.. module:: battery
   :platform: Unix
   :synopsis: Battery behaviours

Behaviours related to checking and reacting to battery notifications.

----

"""

##############################################################################
# Imports
##############################################################################

import py_trees
import rospy
import somanet_msgs.msg as somanet_msgs
import gopher_configuration

from . import recovery

##############################################################################
# Battery
##############################################################################

class CheckChargeState(py_trees.Behaviour):

    def __init__(self, name, expected_state):
        """
        :param int expected_state: return success if the battery enters this charging state
        """
        super(CheckChargeState, self).__init__(name)
        self._battery_subscriber = rospy.Subscriber("~battery", somanet_msgs.SmartBatteryStatus, self.battery_callback)
        self.expected_state = expected_state
        self.charge_state = None

    def initialise(self):
        self.charge_state = None

    def battery_callback(self, msg):
        self.charge_state = msg.charge_state

    def update(self):
        if self.charge_source == None:
            self.feedback_message = "waiting for battery update"
            return py_trees.Status.RUNNING
        elif self.charge_state == self.expected_state:
            return py_trees.Status.SUCCESS
        else:
            return py_trees.Status.FAILURE

class CheckChargeSource(py_trees.Behaviour):

    def __init__(self, name, expected_source):
        """
        :param int expected_source: return success if the battery is being charged from this source
        """
        super(CheckChargeSource, self).__init__(name)
        self._battery_subscriber = rospy.Subscriber("~battery", somanet_msgs.SmartBatteryStatus, self.battery_callback)
        self.expected_source = expected_source
        self.charge_source = None

    def initialise(self):
        self.charge_source = None

    def battery_callback(self, msg):
        self.charge_source = msg.charging_source

    def update(self):
        if self.charge_source == None:
            self.feedback_message = "waiting for battery update"
            return py_trees.Status.RUNNING
        elif self.charge_source == self.expected_source:
            return py_trees.Status.SUCCESS
        else:
            return py_trees.Status.FAILURE

class CheckBatteryLevel(py_trees.Behaviour):
    def __init__(self, name):
        # setup
        super(CheckBatteryLevel, self).__init__(name)
        self.gopher = gopher_configuration.Configuration()
        self.successes = 0
        self.battery_percentage = 100
        rospy.Subscriber("~battery", somanet_msgs.SmartBatteryStatus, self.battery_callback)

    def update(self):
        self.feedback_message = "battery %s%% [low: %s%%]" % (self.battery_percentage, self.gopher.battery.low)
        self.logger.debug("  %s [update()]" % self.name)
        if self.battery_percentage < self.gopher.battery.low:
            if self.successes % 10 == 0:
                rospy.logwarn("Battery is low! I'm outta here!")
            self.successes += 1
            return py_trees.Status.SUCCESS
        else:
            return py_trees.Status.FAILURE

    def battery_callback(self, msg):
        self.battery_percentage = msg.percentage


def create_battery_tree(name):
    check_battery_level = CheckBatteryLevel("Check Battery Level")
    homebase_recovery = recovery.HomebaseRecovery("Homebase Recovery")
    children = [check_battery_level, homebase_recovery]
    # this needs more thought...
    # also need a wait for charge once you're back...
    # children=[check_battery_level, go_home, moveit.Finishing("Finishing")]
    root = py_trees.Sequence(children=children, name=name)
    return root

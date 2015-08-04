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

import logging
import py_trees
import rospy
import somanet_msgs.msg as somanet_msgs
import std_msgs.msg as std_msgs

from . import moveit

##############################################################################
# Battery
##############################################################################


class CheckBatteryLevel(py_trees.Behaviour):
    def __init__(self, name):
        # setup
        super(CheckBatteryLevel, self).__init__(name)
        self.battery_critical_threshold = rospy.get_param("battery/critical_threshold", 30)
        self.battery_percentage = 100
        rospy.Subscriber("~battery", somanet_msgs.SmartBatteryStatus, self.battery_callback)

    def update(self):
        self.feedback_message = "battery %s%%" % self.battery_percentage
        self.logger.debug("  %s [update()]" % self.name)
        if self.battery_percentage < self.battery_critical_threshold:
            return py_trees.Status.SUCCESS
        else:
            return py_trees.Status.FAILURE

    def battery_callback(self, msg):
        self.battery_percentage = msg.percentage


def create_battery_tree(name):
    check_battery_level = CheckBatteryLevel("Check Battery Level")
    go_home = moveit.GoHome("Go Home To Sleep")
    dock = moveit.Dock("Dock")
    root = py_trees.Sequence(children=[check_battery_level, go_home, dock], name=name)
    return root

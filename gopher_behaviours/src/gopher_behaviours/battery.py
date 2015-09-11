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

from . import moveit

##############################################################################
# Battery
##############################################################################


class CheckBatteryLevel(py_trees.Behaviour):
    def __init__(self, name):
        # setup
        super(CheckBatteryLevel, self).__init__(name)
        self.battery_low_threshold = rospy.get_param("battery/low_threshold", 30)
        self.successes = 0
        self.battery_percentage = 100
        rospy.Subscriber("~battery", somanet_msgs.SmartBatteryStatus, self.battery_callback)

    def update(self):
        self.feedback_message = "battery %s%%" % self.battery_percentage
        self.logger.debug("  %s [update()]" % self.name)
        if self.battery_percentage < self.battery_low_threshold:
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
    go_home = moveit.GoHome("Go Home To Sleep")
    face = moveit.RotateToStation("Face Station")
    dock = moveit.DockSelector("Docking Group")
    root = py_trees.Sequence(children=[check_battery_level, go_home, face, dock], name=name)
    return root

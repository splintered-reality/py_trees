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

import gopher_configuration
import py_trees
import rospy
import somanet_msgs.msg as somanet_msgs

from . import interactions
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
        self.expected_state = expected_state
        self.charge_state = None
        self._battery_subscriber = rospy.Subscriber("~battery", somanet_msgs.SmartBatteryStatus, self.battery_callback)

    def battery_callback(self, msg):
        self.charge_state = msg.charge_state

    def update(self):
        if self.charge_state is None:
            self.feedback_message = "waiting for battery update"
            return py_trees.Status.RUNNING
        elif self.charge_state == self.expected_state:
            self.feedback_message = "got expected charge state"
            return py_trees.Status.SUCCESS
        else:
            self.feedback_message = "charge state differed from expected"
            return py_trees.Status.FAILURE


def create_wait_to_be_docked(name="Wait to be Docked"):
    """
    Hooks up a subscriber and waits for docking contact to be detected.
    Flashes for help in the meantime.

    :param str name: behaviour name
    :returns: the behaviour
    :rtype: subscribers.CheckSubscriberVariable
    """
    gopher = gopher_configuration.Configuration(fallback_to_defaults=True)
    wait_to_be_docked = interactions.SendNotification(
        name,
        message='waiting to be docked',
        led_pattern=gopher.led_patterns.humans_i_need_help
    )
    checked_docked_contact = py_trees.CheckSubscriberVariable(
        name="Check Docked Contact",
        topic_name=gopher.topics.battery,
        topic_type=somanet_msgs.SmartBatteryStatus,
        variable_name="charging_source",
        expected_value=somanet_msgs.SmartBatteryStatus.CHARGING_SOURCE_DOCK,
        fail_if_no_data=False,
        fail_if_bad_comparison=False,
        clearing_policy=py_trees.common.ClearingPolicy.NEVER
    )
    wait_to_be_docked.add_child(checked_docked_contact)
    return wait_to_be_docked


def create_is_docked(name="Is Docked?"):
    """
    Hooks up a subscriber and checks that the charging source is the dock.

    :param str name: behaviour name
    :returns: the behaviour
    :rtype: subscribers.CheckSubscriberVariable
    """
    gopher = gopher_configuration.Configuration(fallback_to_defaults=True)

    check_docked = py_trees.CheckSubscriberVariable(
        name=name,
        topic_name=gopher.topics.battery,
        topic_type=somanet_msgs.SmartBatteryStatus,
        variable_name="charging_source",
        expected_value=somanet_msgs.SmartBatteryStatus.CHARGING_SOURCE_DOCK,
        fail_if_no_data=False,
        fail_if_bad_comparison=True,
        clearing_policy=py_trees.common.ClearingPolicy.NEVER
    )
    return check_docked


def create_is_discharging(name="Is Discharging?"):
    """
    Hooks up a subscriber and checks that there is no current charging source.
    This will 'block', i.e. return RUNNING until it detects that it is discharging.

    :param str name: behaviour name
    :rtype: subscribers.CheckSubscriberVariable
    """
    gopher = gopher_configuration.Configuration(fallback_to_defaults=True)
    is_discharging = py_trees.CheckSubscriberVariable(
        name=name,
        topic_name=gopher.topics.battery,
        topic_type=somanet_msgs.SmartBatteryStatus,
        variable_name="charging_source",
        expected_value=somanet_msgs.SmartBatteryStatus.CHARGING_SOURCE_NONE,
        fail_if_no_data=False,
        fail_if_bad_comparison=True,
        clearing_policy=py_trees.common.ClearingPolicy.NEVER
    )
    return is_discharging


def create_wait_to_be_unplugged(name="Unplug Me"):
    """
    Hooks up a subscriber and checks that there is no current charging source.
    This will 'block', i.e. return RUNNING until it detects that it is discharging.

    :param str name: subtree root name
    :returns: the subtree
    """
    gopher = gopher_configuration.Configuration(fallback_to_defaults=True)
    wait_for_discharging = py_trees.CheckSubscriberVariable(
        name="Wait for Discharging",
        topic_name=gopher.topics.battery,
        topic_type=somanet_msgs.SmartBatteryStatus,
        variable_name="charging_source",
        expected_value=somanet_msgs.SmartBatteryStatus.CHARGING_SOURCE_NONE,
        fail_if_no_data=False,
        fail_if_bad_comparison=False,
        clearing_policy=py_trees.common.ClearingPolicy.NEVER
    )
    wait_to_be_unplugged = interactions.SendNotification(
        name=name,
        message='waiting to be unplugged',
        led_pattern=gopher.led_patterns.humans_i_need_help
    )
    wait_to_be_unplugged.add_child(wait_for_discharging)
    return wait_to_be_unplugged


class CheckBatteryLevel(py_trees.Behaviour):
    def __init__(self, name):
        # setup
        super(CheckBatteryLevel, self).__init__(name)
        self.gopher = gopher_configuration.Configuration()
        self.successes = 0
        self.battery_percentage = 100
        rospy.Subscriber("~battery", somanet_msgs.SmartBatteryStatus, self.battery_callback)

    def update(self):
        # Note : battery % in the feedback message causes spam in behaviour
        # tree rqt viewer, we are merely interested in the transitions, so do not send this
        # self.feedback_message = "battery %s%% [low: %s%%]" % (self.battery_percentage, self.gopher.battery.low)
        self.logger.debug("  %s [update()]" % self.name)
        if self.battery_percentage < self.gopher.battery.low:
            if self.successes % 10 == 0:  # throttling
                rospy.logwarn("Behaviours [%s]: battery level is low!" % self.name)
            self.successes += 1
            self.feedback_message = "Battery level is low."
            return py_trees.Status.SUCCESS
        else:
            self.feedback_message = "Battery level is ok."
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
    root = py_trees.Sequence(name=name, children=children)
    return root

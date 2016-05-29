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
"""

##############################################################################
# Imports
##############################################################################

import gopher_configuration
import gopher_std_msgs.srv as gopher_std_srvs
import operator
import py_trees
import rospy
import gopher_core_msgs.msg as gopher_core_msgs

from . import interactions

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
        self._battery_subscriber = rospy.Subscriber("~battery", gopher_core_msgs.BatteryState, self.battery_callback)

    def battery_callback(self, msg):
        self.charge_state = msg.sensor_state.power_supply_status

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


def create_check_charging_source(name="Is Docked?",
                                 charging_source=gopher_core_msgs.ChargingSource.DOCK,
                                 clear_every_time=True
                                 ):
    """
    Checks the blackboard to see what the charging source is.

    :param str name: behaviour name
    :param gopher_core_msgs.ChargingSource charging_source: source type to check for (docked, jacked, none)
    :param bool clear_every_time: clear the result every time it runs
    :returns: the behaviour
    :rtype: subscribers.CheckSubscriberVariable
    """
    check_source = py_trees.CheckBlackboardVariable(
        name=name,
        variable_name="battery.charging_source.source",
        expected_value=charging_source,
        comparison_operator=operator.eq,
        clearing_policy=py_trees.common.ClearingPolicy.ON_INITIALISE if clear_every_time else py_trees.common.ClearingPolicy.NEVER
    )
    return check_source


def create_is_docked():
    """ Check the recorded (blackboard) battery state to see if it is docked. """
    return create_check_charging_source(name="Is Docked?", charging_source=gopher_core_msgs.ChargingSource.DOCK, clear_every_time=True)


def create_is_jacked():
    """Check the recorded (blackboard) battery state to see if it is jacked."""
    return create_check_charging_source(name="Is Jacked?", charging_source=gopher_core_msgs.ChargingSource.JACK, clear_every_time=True)


def create_is_discharging():
    """ Check the recorded (blackboard) battery state to see if it is discharging."""
    return create_check_charging_source(name="Is Discharging?", charging_source=gopher_core_msgs.ChargingSource.NONE, clear_every_time=True)


def create_was_docked():
    """ Check the recorded (blackboard) battery state to see if it was docked. """
    return create_check_charging_source(name="Was Docked?", charging_source=gopher_core_msgs.ChargingSource.DOCK, clear_every_time=False)


def create_was_jacked():
    """Check the recorded (blackboard) battery state to see if it was jacked."""
    return create_check_charging_source(name="Was Jacked?", charging_source=gopher_core_msgs.ChargingSource.JACK, clear_every_time=False)


def create_was_discharging():
    """ Check the recorded (blackboard) battery state to see if it was discharging."""
    return create_check_charging_source(name="Was Discharging?", charging_source=gopher_core_msgs.ChargingSource.NONE, clear_every_time=False)


def create_wait_to_be_unplugged(name="Unplug Me"):
    """
    Hooks up a subscriber and checks that there is no current charging source.
    This will 'block', i.e. return RUNNING until it detects that it is discharging.

    :param str name: subtree root name
    :returns: the subtree
    """
    gopher = gopher_configuration.configuration.Configuration(fallback_to_defaults=True)

    wait_to_be_unplugged = py_trees.composites.Parallel(
        name="Wait to be Unplugged",
        policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE
    )
    wait_to_be_unplugged.blackbox_level = py_trees.common.BlackBoxLevel.DETAIL
    flash_notification = interactions.Notification(
        name='Flash for Help',
        message='waiting for button press to continue',
        led_pattern=gopher.led_patterns.humans_i_need_help,
        duration=gopher_std_srvs.NotifyRequest.INDEFINITE
    )
    check_if_discharging = py_trees.CheckSubscriberVariable(
        name="Check if Discharging",
        topic_name=gopher.topics.battery,
        topic_type=gopher_core_msgs.BatteryState,
        variable_name="charging_source.source",
        expected_value=gopher_core_msgs.ChargingSource.NONE,
        fail_if_no_data=False,
        fail_if_bad_comparison=False,
        clearing_policy=py_trees.common.ClearingPolicy.NEVER
    )
    wait_to_be_unplugged.add_child(flash_notification)
    wait_to_be_unplugged.add_child(check_if_discharging)
    return wait_to_be_unplugged


class ToBlackboard(py_trees.subscribers.ToBlackboard):
    """
    Subscribes to the battery message and writes battery data to the blackboard.
    Also adds a qualitative low/high to the blackboard that buffers against ping-pong
    around the threshold.

    This behaviour returns RUNNING if it got no data, SUCCESS otherwise.

    Blackboard Variables:

     * battery             (w) [somanet_msgs.Smartbattery] : raw battery message
     * battery_low_warning (w) [bool]                      : False if battery is ok, True if critically low
    """
    def __init__(self, name, topic_name):
        super(ToBlackboard, self).__init__(name=name,
                                           topic_name=topic_name,
                                           topic_type=gopher_core_msgs.BatteryState,
                                           blackboard_variables={"battery": None},
                                           clearing_policy=py_trees.common.ClearingPolicy.NEVER
                                           )
        self.successes = 0
        self.blackboard = py_trees.blackboard.Blackboard()
        self.blackboard.battery = gopher_core_msgs.BatteryState()
        self.blackboard.battery_low_warning = False   # decision making
        self.gopher = gopher_configuration.configuration.Configuration(fallback_to_defaults=True)

    def update(self):
        self.logger.debug("  %s [ToBlackboard.update()]" % self.name)
        status = super(ToBlackboard, self).update()
        if status != py_trees.common.Status.RUNNING:
            # we got something
            if self.blackboard.battery.sensor_state.percentage < self.gopher.battery.low:
                if self.successes % 10 == 0:  # throttling, just in case we have an up/down battery percentage incoming
                    self.blackboard.battery_low_warning = True
                    rospy.logwarn("Behaviours [%s]: battery level is low!" % self.name)
                self.successes += 1
                self.feedback_message = "Battery level is low."
            else:
                self.blackboard.battery_low_warning = False
                self.feedback_message = "Battery level is ok."
        return status

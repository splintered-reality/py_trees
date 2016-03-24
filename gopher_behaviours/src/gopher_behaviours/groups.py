#!/usr/bin/env python
#
# License: Yujin
#   https://raw.github.com/yujinrobot/gopher_crazy_hospital/license/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
.. module:: groups
   :platform: Unix
   :synopsis: Subtrees that meld multiple behaviours into a single cohesive functionality.

----
"""
##############################################################################
# Imports
##############################################################################

import py_trees
import somanet_msgs.msg as somanet_msgs
import gopher_configuration

from . import interactions
from . import battery

# from .dock import Dock
from .park import Park
# from .undock import Undock
from .unpark import UnPark

##############################################################################
# Black Boxes
##############################################################################


class Starting(py_trees.Selector):
    def __init__(self, name):
        undockseq = py_trees.Sequence("Undock", [battery.create_check_docked_behaviour(),
                                                 Undock("Undock"), py_trees.Pause("Pause", 2)])
        unparkseq = py_trees.Sequence("UnPark", [UnPark("UnPark"), py_trees.Pause("Pause", 2)])
        children = [undockseq, unparkseq]
        super(Starting, self).__init__(name, children)


class Finishing(py_trees.Selector):
    def __init__(self, name):
        self.gopher = gopher_configuration.Configuration()

        dock_notify = interactions.SendNotification("Notify", self.gopher.led_patterns.humans_i_am_done,
                                                    message="Successfully docked.", cancel_on_stop=False, duration=5)
        dock_notify.add_child(interactions.Articulate("Yawn", self.gopher.sounds.done))

        park_notify = interactions.SendNotification("Notify", self.gopher.led_patterns.humans_i_am_done,
                                                    message="Successfully parked.", cancel_on_stop=False, duration=5)
        dock_notify.add_child(interactions.Articulate("Yawn", self.gopher.sounds.done))

        dockseq = py_trees.Sequence("Dock/notify", [Dock("Dock"), dock_notify])
        parkseq = py_trees.Sequence("Park/notify", [Park("Park"), park_notify])

        wait_for_charge_confirm = interactions.SendNotification("Wait for Jack/Dock",
                                                                led_pattern=self.gopher.led_patterns.humans_i_need_help,
                                                                message="waiting for charge or confirm button press")
        recovery_selector = py_trees.Selector("Charge/Confirm", [
            battery.CheckChargeState("Charging?", somanet_msgs.SmartBatteryStatus.CHARGING),
            interactions.CheckButtonPressed("Confirm pressed?", self.gopher.buttons.go)
        ])
        charge_confirm_condition = py_trees.behaviours.Condition('Wait for recovery', recovery_selector, py_trees.Status.SUCCESS)
        wait_for_charge_confirm.add_child(charge_confirm_condition)

        children = [parkseq, dockseq, wait_for_charge_confirm]
        super(Finishing, self).__init__(name, children)

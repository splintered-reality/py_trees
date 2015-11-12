#!/usr/bin/env python

import py_trees
from . import dockit
from . import parkit
from . import interactions
from . import battery
from . import blackboard_handlers
from .time import Pause
from gopher_std_msgs.msg import Notification
from somanet_msgs.msg import SmartBatteryStatus
import gopher_configuration

class Starting(py_trees.Selector):
    def __init__(self, name):
        undockseq = py_trees.Sequence("Undock", [battery.CheckChargeSource("Check for dock",
                                                                           SmartBatteryStatus.CHARGING_SOURCE_DOCK),
                                                 dockit.Undock("Undock"), Pause("Pause", 2)])
        unparkseq = py_trees.Sequence("Unpark", [parkit.Unpark("Unpark"), Pause("Pause", 2)])
        children = [undockseq, unparkseq]
        super(Starting, self).__init__(name, children)


class Finishing(py_trees.Selector):
    def __init__(self, name):
        self.gopher = gopher_configuration.Configuration()

        dock_notify = interactions.SendNotification("Notify", led_pattern=Notification.FLASH_GREEN,
                                                    message="Successfully docked.", cancel_on_stop=False)
        dock_notify.add_child(py_trees.behaviours.Success())

        park_notify = interactions.SendNotification("Notify", led_pattern=Notification.FLASH_GREEN, 
                                                    message="Successfully parked.", cancel_on_stop=False)
        park_notify.add_child(py_trees.behaviours.Success())

        dodock = py_trees.Sequence("Dock/notify", [dockit.Dock("Dock"), dock_notify,
                                                   interactions.Articulate("Yawn", self.gopher.sounds.done)
                                                   ]
                                   )
        dockseq = py_trees.Sequence("Maybe dock", [blackboard_handlers.CheckBlackboardVariable("Was I docked?", 'parked', False),
                                                   dodock])

        dopark = py_trees.Sequence("Park/notify", [parkit.Park("Park"), park_notify,
                                                   interactions.Articulate("Yawn", self.gopher.sounds.done)
                                                   ]
                                   )
        parkseq = py_trees.Sequence("Maybe park", [blackboard_handlers.CheckBlackboardVariable("Was I parked?", 'parked', True),
                                                   dopark])

        wait_for_charge_confirm = interactions.SendNotification("Wait for Jack/Dock", 
                                                                led_pattern=self.gopher.led_patterns.humans_i_need_help,
                                                                message="waiting for charge or confirm button press")
        charge_confirm_selector = py_trees.Selector("Charge/Confirm", [battery.CheckChargeState("Charging?",
                                                                                                SmartBatteryStatus.CHARGING),
                                                                       interactions.CheckButtonPressed("Confirm pressed?",
                                                                                                       self.gopher.buttons.go)
                                                                       ]
                                                    )
        
        wait_for_charge_confirm.add_child(charge_confirm_selector)
        children = [parkseq, dockseq, wait_for_charge_confirm]
        super(Finishing, self).__init__(name, children)

#
# License: Yujin
#
##############################################################################
# Description
##############################################################################

"""
.. module:: recovery
   :platform: Unix
   :synopsis: Recovery behaviours

Oh my spaghettified magnificence,
Bless my noggin with a tickle from your noodly appendages!

----

"""

##############################################################################
# Imports
##############################################################################

import gopher_configuration
import gopher_navi_msgs.msg as gopher_navi_msgs
import py_trees

from . import battery
from . import interactions
from . import navigation
from . import park

##############################################################################
# Behaviours
##############################################################################


class HomebaseRecovery(py_trees.Sequence):
    def __init__(self, name):
        """
        WARNING: this is not a very complete recovery behaviour. It expects the
        user to go all the way back to homebase and press the button. But doesn't
        inform him exactly that he should do this.

        :param str name: behaviour name
        """
        super(HomebaseRecovery, self).__init__(name)
        self.gopher = gopher_configuration.Configuration(fallback_to_defaults=True)
        flash_leds = interactions.SendNotification("Flash - I Need Help",
                                                   led_pattern=self.gopher.led_patterns.humans_i_need_help,
                                                   message="homebase recovery - need human assistance to teleop home")
        wait_for_button = interactions.create_wait_for_go_button(name="Teleop -> Homebase and Hit the Green Button!")
        teleport = navigation.Teleport("Activate the Homebase Teleport!",
                                       gopher_navi_msgs.TeleportGoal(location="homebase", special_effects=True)
                                       )
        parking = park.Park("Park")
        flash_leds.add_child(wait_for_button)
        self.add_child(flash_leds)
        self.add_child(teleport)
        self.add_child(parking)


def create_battery_recovery_tree(name):
    check_battery_level = battery.CheckBatteryLevel("Check Battery Level")
    homebase_recovery = HomebaseRecovery("Homebase Recovery")
    children = [check_battery_level, homebase_recovery]
    # this needs more thought...
    # also need a wait for charge once you're back...
    # children=[check_battery_level, go_home, moveit.Finishing("Finishing")]
    root = py_trees.Sequence(name=name, children=children)
    return root

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

##############################################################################
# Behaviours
##############################################################################


def create_homebase_recovery_children():
    """
    Create and return the required children to do a homebase recovery.

    :returns: children for a homebase recovery (append to a sequence)

    .. warning::

       This is not very complete, and is a fully manual recovery. It expects the user
       to go all the way back to homebase and press the button but doesn't inform him
       exactly that he should do this.
    """
    gopher = gopher_configuration.Configuration(fallback_to_defaults=True)
    flash_leds = interactions.SendNotification("Flash - I Need Help",
                                               led_pattern=gopher.led_patterns.humans_i_need_help,
                                               message="homebase recovery - need human assistance to teleop home")
    wait_for_button = interactions.create_wait_for_go_button(name="Teleop -> Homebase and Hit the Green Button!")
    teleport = navigation.Teleport("Activate the Homebase Teleport!",
                                   gopher_navi_msgs.TeleportGoal(location="homebase", special_effects=True)
                                   )
    flash_leds.add_child(wait_for_button)
    return [flash_leds, teleport]


def create_battery_recovery_tree(name):
    check_battery_level = battery.CheckBatteryLevel("Check Battery Level")
    # this needs more thought...
    # also need a wait for charge once you're back...
    # children=[check_battery_level, go_home, moveit.Finishing("Finishing")]
    root = py_trees.Sequence(name=name)
    root.add_child(check_battery_level)
    root.add_children(create_homebase_recovery_children())
    return root

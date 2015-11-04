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

from . import interactions
from . import navigation

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
        self.gopher = gopher_configuration.Configuration()
        flash_leds = interactions.FlashLEDs("Flash - I Need Help",
                                            led_pattern=self.gopher.led_patterns.humans_i_need_help,
                                            message="homebase recovery - need human assistance to teleop home")
        wait_for_button = interactions.WaitForButton("Teleop -> Homebase and Hit the Green Button!", self.gopher.buttons.go)
        teleport = navigation.Teleport("Activate the Homebase Teleport!",
                                       gopher_navi_msgs.TeleportGoal(location="homebase")
                                       )
        flash_leds.add_child(wait_for_button)
        self.add_child(flash_leds)
        self.add_child(teleport)

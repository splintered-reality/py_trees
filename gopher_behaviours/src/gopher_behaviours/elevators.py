#
# License: Yujin
#
##############################################################################
# Description
##############################################################################

"""
.. module:: elevators
   :platform: Unix
   :synopsis: Elevator related behaviours

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
from . import recovery

##############################################################################
# Behaviours
##############################################################################


class HumanAssistedElevators(py_trees.Selector):

    def __init__(self, name, origin, elevator, destination):
        """
        :param str name: behaviour name
        :param str origin: semantic world (unique_name) that it is coming from.
        :param str destination: semantic world (unique_name) that it is going to (another floor).
        :param gopher_semantic_msgs.Elevator elevator: the bridge between worlds
        """
        super(HumanAssistedElevators, self).__init__(name)
        self.origin = origin
        self.destination = destination
        self.elevator = elevator
        self.elevator_level_origin = _get_elevator_level(elevator, origin)
        self.elevator_level_destination = _get_elevator_level(elevator, destination)
        self.gopher = gopher_configuration.Configuration()

        ###########################################
        # Tree
        ###########################################
        # Assume all things here...as both semantics and planner should already validate everything.
        elevator_sequence = _generate_elevator_sequence(self.gopher, self.elevator.unique_name, self.elevator_level_origin, self.elevator_level_destination)
        failure_sequence = recovery.HomebaseRecovery("Failure")
        self.add_child(elevator_sequence)
        self.add_child(failure_sequence)


def _generate_elevator_sequence(gopher_configuration, elevator_name, elevator_level_origin, elevator_level_destination):
    """
    :param gopher_configuration.Configuration gopher_configuration:
    :param str elevator_name: unique name of the elevator
    :param gopher_semantic_msgs.ElevatorLevel elevator_level_origin:
    :param gopher_semantic_msgs.ElevatorLevel elevator_level_destination:
    """
    elevator_sequence = py_trees.Sequence("Sequence")

    # DJS : might need a delivery free MoveToGoal here
    move_to_elevator = navigation.MoveIt("To Elevator", elevator_level_origin.entry)
    honk = interactions.Articulate("Honk", gopher_configuration.sounds.honk)
    telesound = interactions.Articulate("Transporter sound", gopher_configuration.sounds.teleport)
    flash_leds_travelling = interactions.FlashLEDs("Travelling", led_pattern=gopher_configuration.led_patterns.humans_give_me_input)
    flash_leds_travelling.add_child(interactions.WaitForButton("Wait for Button", gopher_configuration.buttons.go))

    flash_leds_teleporting = interactions.FlashLEDs("Teleporting", led_pattern=gopher_configuration.led_patterns.holding)

    goal = gopher_navi_msgs.TeleportGoal()
    goal.elevator_location = gopher_navi_msgs.ElevatorLocation()
    goal.elevator_location.elevator = elevator_name
    goal.elevator_location.world = elevator_level_destination.world
    goal.elevator_location.location = gopher_navi_msgs.ElevatorLocation.EXIT
    elevator_teleport = navigation.Teleport("Elevator Teleport", goal)
    flash_leds_teleporting.add_child(telesound)
    flash_leds_teleporting.add_child(elevator_teleport)

    elevator_sequence.add_child(move_to_elevator)
    elevator_sequence.add_child(honk)
    elevator_sequence.add_child(flash_leds_travelling)
    elevator_sequence.add_child(flash_leds_teleporting)
    return elevator_sequence


def _get_elevator_level(elevator, world):
    for level in elevator.levels:
        if level.world == world:
            return level
    return None

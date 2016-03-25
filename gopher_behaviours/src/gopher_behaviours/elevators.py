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
import std_msgs.msg as std_msgs

from . import interactions
from . import navigation
from . import recovery

##############################################################################
# Behaviours
##############################################################################


class Elevators(py_trees.Sequence):
    pass


class HumanAssistedElevators(Elevators):

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
        # use the add_child function to make sure that the parents are correctly initialised
        map(self.add_child, _generate_elevator_children(self.gopher, self.elevator.unique_name, self.elevator_level_origin, self.elevator_level_destination))


def _generate_elevator_children(gopher_configuration, elevator_name, elevator_level_origin, elevator_level_destination):
    """
    :param gopher_configuration.Configuration gopher_configuration:
    :param str elevator_name: unique name of the elevator
    :param gopher_semantic_msgs.ElevatorLevel elevator_level_origin:
    :param gopher_semantic_msgs.ElevatorLevel elevator_level_destination:
    """
    move_to_elevator = navigation.MoveIt("To Elevator", elevator_level_origin.entry)
    honk = interactions.Articulate("Honk", gopher_configuration.sounds.honk)
    telesound = interactions.Articulate("Transporter sound", gopher_configuration.sounds.teleport)
    flash_leds_travelling = interactions.SendNotification("Travelling", led_pattern=gopher_configuration.led_patterns.humans_give_me_input, button_confirm=True, message="Waiting for confirm button press in front of elevator")
    flash_leds_travelling.add_child(py_trees.subscribers.WaitForSubscriberData(name="Wait for Button", topic_name=gopher_configuration.buttons.go, topic_type=std_msgs.String))

    flash_leds_teleporting = interactions.SendNotification("Teleporting", led_pattern=gopher_configuration.led_patterns.im_doing_something_cool, message="teleporting from {0} to {1}".format(elevator_level_origin, elevator_level_destination))

    goal = gopher_navi_msgs.TeleportGoal()
    goal.elevator_location = gopher_navi_msgs.ElevatorLocation()
    goal.elevator_location.elevator = elevator_name
    goal.elevator_location.world = elevator_level_destination.world
    goal.elevator_location.location = gopher_navi_msgs.ElevatorLocation.EXIT
    goal.special_effects = False  # we do our own special effects
    elevator_teleport = navigation.Teleport("Elevator Teleport", goal)
    flash_leds_teleporting.add_child(telesound)
    flash_leds_teleporting.add_child(elevator_teleport)

    children = []
    children.append(move_to_elevator)
    children.append(honk)
    children.append(flash_leds_travelling)
    children.append(flash_leds_teleporting)
    return children


def _get_elevator_level(elevator, world):
    for level in elevator.levels:
        if level.world == world:
            return level
    return None

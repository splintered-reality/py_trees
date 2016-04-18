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
import gopher_std_msgs.msg as gopher_std_msgs
import operator
import rospy
import std_msgs.msg as std_msgs

from . import elf
from . import interactions
from . import navigation

##############################################################################
# Behaviours
##############################################################################


class Elevators(py_trees.Sequence):
    pass


class HumanAssistedElevators(Elevators):

    def __init__(self, name, origin, elevator, destination, elf_initialisation_type=elf.InitialisationType.TELEOP):
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
        self.elf_initialisation_type = elf_initialisation_type

        ###########################################
        # Tree
        ###########################################
        # Assume all things here...as both semantics and planner should already validate everything.
        # use the add_child function to make sure that the parents are correctly initialised
        children = _generate_elevator_children(
            self.gopher,
            self.elevator.unique_name,
            self.elevator_level_origin,
            self.elevator_level_destination,
            self.elf_initialisation_type
        )
        if not children:
            rospy.logerr("Behaviours [%s]: failed to generate the required elevator children" % self.name)
        else:
            map(self.add_child, children)


def _generate_elevator_children(
        gopher_configuration,
        elevator_name,
        elevator_level_origin,
        elevator_level_destination,
        elf_initialisation_type
        ):
    """
    :param gopher_configuration.Configuration gopher_configuration:
    :param str elevator_name: unique name of the elevator
    :param gopher_semantic_msgs.ElevatorLevel elevator_level_origin:
    :param gopher_semantic_msgs.ElevatorLevel elevator_level_destination:
    :param elf.InitialisationType elf_initialisation_type:
    """
    move_to_elevator = navigation.MoveIt("To Elevator '%s'" % elevator_name, elevator_level_origin.entry)
    goal_finishing = navigation.GoalFinishing(name="Finish '%s'" % elevator_name, goal_pose=elevator_level_origin.entry)
    honk = interactions.Articulate("Honk", gopher_configuration.sounds.honk)

    travelling = py_trees.composites.Parallel(
        name="Travelling",
        policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE
    )

    flash_leds_travelling = interactions.Notification(
        name="Flash for Input",
        led_pattern=gopher_configuration.led_patterns.humans_i_need_help,
        button_confirm=gopher_std_msgs.Notification.BUTTON_ON,
        button_cancel=gopher_std_msgs.Notification.RETAIN_PREVIOUS,
        message="Waiting for confirm button press in front of elevator")
    wait_for_go_button_travelling = py_trees.blackboard.WaitForBlackboardVariable(
        name="Wait for Button",
        variable_name="event_go_button",
        expected_value=True,
        comparison_operator=operator.eq,
        clearing_policy=py_trees.common.ClearingPolicy.ON_INITIALISE
    )

    goal = gopher_navi_msgs.TeleportGoal()
    goal.elevator_location = gopher_navi_msgs.ElevatorLocation()
    goal.elevator_location.elevator = elevator_name
    goal.elevator_location.world = elevator_level_destination.world
    goal.elevator_location.location = gopher_navi_msgs.ElevatorLocation.EXIT
    goal.special_effects = False  # we do our own special effects

    travelling.add_child(flash_leds_travelling)
    travelling.add_child(wait_for_go_button_travelling)

    teleporting = py_trees.composites.Parallel(
        name="Teleport to '%s'" % elevator_level_destination.world,
        policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE
    )
    # teleporting_jobs = py_trees.composites.Sequence("Teleport Jobs")
    flash_leds_teleporting = interactions.Notification(
        name="Flash Cool",
        led_pattern=gopher_configuration.led_patterns.im_doing_something_cool,
        sound=gopher_configuration.sounds.teleport,
        message="teleporting from {0} to {1}".format(elevator_level_origin, elevator_level_destination)
    )
    elevator_teleport = navigation.Teleport("Elevator Teleport", goal)

    teleporting.add_child(flash_leds_teleporting)
    teleporting.add_child(elevator_teleport)

    if elf_initialisation_type == elf.InitialisationType.TELEOP:
        elf_initialisation = None
    elif elf_initialisation_type == elf.InitialisationType.AR:
        elf_initialisation = elf.ARInitialisation()
        elf_reset = elf.Reset()
    else:
        return []

    children = []
    children.append(move_to_elevator)
    children.append(goal_finishing)
    children.append(honk)
    children.append(travelling)
    children.append(teleporting)
    if elf_initialisation is not None:
        children.append(elf_reset)
        children.append(elf_initialisation)
    return children


def _get_elevator_level(elevator, world):
    for level in elevator.levels:
        if level.world == world:
            return level
    return None

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
import elevator_interactions_msgs.msg as elevator_interactions_msgs
import enum
import gopher_configuration
import gopher_navi_msgs.msg as gopher_navi_msgs
import py_trees
import gopher_semantics
import gopher_std_msgs.msg as gopher_std_msgs
import operator
import rospy
import std_msgs.msg as std_msgs
import uuid

from . import elevator_operator_client
from . import elf
from . import interactions
from . import navigation
from . import utilities

from gopher_behaviours.cfg import QuirkyDeliveriesConfig

##############################################################################
# Helpers
##############################################################################


def get_default_args_from_semantics():
    """
    For use with documentation generating tooling to provide default, usable args.
    """
    semantics = gopher_semantics.Semantics(fallback_to_defaults=True)
    origin = semantics.worlds.default
    for world in semantics.worlds:
        if world != origin:
            destination = world
    for e in semantics.elevators.values():
        for level in e.levels:
            if level.world == origin:
                elevator = e
                break
    return (origin, elevator, destination)


def get_elevator_level(elevator, world):
    for level in elevator.levels:
        if level.world == world:
            return level
    return None

##############################################################################
# Enum
##############################################################################


class InteractionType(enum.IntEnum):
    """ An enumerator representing the types of interaction with the elevator operator """

    HUMAN_ASSISTED = 0
    """Human in control; no interaction with the elevator operator """
    PARTIAL_ASSISTED = 1
    """Elevator operator controlling the elevator; human moving the robot in and out"""
    AUTONOMOUS = 2
    """Elevator operator and robot on their own"""


string_to_interaction_type = {
    QuirkyDeliveriesConfig.QuirkyDeliveries_human: InteractionType.HUMAN_ASSISTED,
    QuirkyDeliveriesConfig.QuirkyDeliveries_partial: InteractionType.PARTIAL_ASSISTED,
    QuirkyDeliveriesConfig.QuirkyDeliveries_autonomous: InteractionType.AUTONOMOUS,
}

##############################################################################
# Subtrees
##############################################################################


def create_move_to_elevator_subtree(elevator_name, elevator_level_entry_pose):
    """
    :param str elevator_name: name of the elevator as defined in semantics
    :param dict elevator_level_entry_pose: semantics style pose type for the move to/finishing goal ({x, y, theta})
    """
    gopher = gopher_configuration.configuration.Configuration(fallback_to_defaults=True)
    move_to_elevator = py_trees.composites.Sequence(name="Moving There")
    move_to = navigation.MoveIt("Move To\n%s" % utilities.to_human_readable(elevator_name), elevator_level_entry_pose)
    goal_finishing = navigation.GoalFinishing(name="Finish", goal_pose=elevator_level_entry_pose)
    honk = interactions.Articulate("Honk", gopher.sounds.honk)
    move_to_elevator.add_children([move_to, goal_finishing, honk])
    return move_to_elevator


def create_teleport_and_initialise_children(
        elevator_name,
        elevator_level_origin,
        elevator_level_destination,
        elf_initialisation_type):
    """
    Note that the teleop elf initialisation type is not necessary - the teleport directly does the same job, so
    no teleelf is attached to the subtree.

    :param elevator_level_origin:
    :param elevator_level_destination:
    :param elf.InitialisationType elf_initialisation_type: the kind of initialisation required at the other end

    :returns: a list of behaviours to be added as children to a parent sequence

    :raises ValueError: if elf initialisation type is incorrect
    """
    gopher = gopher_configuration.Configuration(fallback_to_defaults=True)
    goal = gopher_navi_msgs.TeleportGoal()
    goal.elevator_location = gopher_navi_msgs.ElevatorLocation()
    goal.elevator_location.elevator = elevator_name
    goal.elevator_location.world = elevator_level_destination.world
    goal.elevator_location.location = gopher_navi_msgs.ElevatorLocation.EXIT
    goal.special_effects = False  # we do our own special effects

    teleporting = py_trees.composites.Parallel(
        name="Teleport",
        policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE
    )
    # teleporting_jobs = py_trees.composites.Sequence("Teleport Jobs")
    flash_leds_teleporting = interactions.Notification(
        name="Flash Cool",
        led_pattern=gopher.led_patterns.im_doing_something_cool,
        sound=gopher.sounds.teleport,
        message="teleporting from {0} to {1}".format(elevator_level_origin, elevator_level_destination)
    )
    teleport = navigation.Teleport("Teleport to %s" % utilities.to_human_readable(elevator_level_destination.world), goal)

    if elf_initialisation_type == elf.InitialisationType.TELEOP:
        elf_initialisation = None
    elif elf_initialisation_type == elf.InitialisationType.AR:
        elf_initialisation = elf.ARInitialisation()
        elf_pause = py_trees.timers.Timer(name="Pause", duration=1.0)
        elf_reset = elf.Reset()
    else:
        raise ValueError("Incorrect elf initialisation type")

    #################################
    # Blackboxes
    #################################
    teleporting.blackbox_level = py_trees.common.BlackBoxLevel.DETAIL
    if elf_initialisation is not None:
        elf_initialisation.blackbox_level = py_trees.common.BlackBoxLevel.DETAIL

    #################################
    # Children
    #################################
    teleporting.add_child(flash_leds_teleporting)
    teleporting.add_child(teleport)
    return [teleporting, elf_pause, elf_reset, elf_initialisation] if elf_initialisation else [teleporting]


##############################################################################
# Behaviours
##############################################################################


class Elevators(py_trees.Sequence):
    pass


class HumanAssistedElevators(Elevators):

    def __init__(self, name="Human Assisted Elevators", origin=None, elevator=None, destination=None, elf_initialisation_type=elf.InitialisationType.TELEOP):
        """
        :param str name: behaviour name
        :param str origin: semantic world (unique_name) that it is coming from.
        ;param gopher_semantic_msgs.Elevator
        :param str destination: semantic world (unique_name) that it is going to (another floor).
        :param gopher_semantic_msgs.Elevator elevator: the bridge between worlds
        """
        super(HumanAssistedElevators, self).__init__(name)
        if not origin or not elevator or not destination:
            (origin, elevator, destination) = get_default_args_from_semantics()
        self.origin = origin
        self.destination = destination
        self.elevator = elevator
        self.elevator_level_origin = get_elevator_level(elevator, origin)
        self.elevator_level_destination = get_elevator_level(elevator, destination)
        self.gopher = gopher_configuration.Configuration(fallback_to_defaults=True)
        self.elf_initialisation_type = elf_initialisation_type

        ###########################################
        # Tree
        ###########################################
        # Assume all things here...as both semantics and planner should already validate everything.
        # use the add_child function to make sure that the parents are correctly initialised
        children = self._generate_elevator_children(self.gopher,
                                                    self.elevator.unique_name,
                                                    self.elevator_level_origin,
                                                    self.elevator_level_destination,
                                                    self.elf_initialisation_type)
        if not children:
            rospy.logerr("Behaviours [%s]: failed to generate the required elevator children" % self.name)
        else:
            map(self.add_child, children)

    def _generate_elevator_children(self,
                                    gopher_configuration,
                                    elevator_name,
                                    elevator_level_origin,
                                    elevator_level_destination,
                                    elf_initialisation_type):
        """
        :param gopher_configuration.Configuration gopher_configuration:
        :param str elevator_name: unique name of the elevator
        :param gopher_semantic_msgs.ElevatorLevel elevator_level_origin:
        :param gopher_semantic_msgs.ElevatorLevel elevator_level_destination:
        :param elf.InitialisationType elf_initialisation_type:
        """

        move_to_elevator = create_move_to_elevator_subtree(elevator_name, elevator_level_origin.entry)

        ride = interactions.flash_and_wait_for_go_button(
            name="Ride",
            notification_behaviour_name="Flash for Input",
            led_pattern=gopher_configuration.led_patterns.humans_give_me_input,
            message="Waiting for confirm button press in front of elevator"
        )

        try:
            teleport_and_initialise_children = create_teleport_and_initialise_children(
                elevator_name=elevator_name,
                elevator_level_origin=elevator_level_origin,
                elevator_level_destination=elevator_level_destination,
                elf_initialisation_type=elf_initialisation_type)
        except ValueError as e:
            raise e

        #################################
        # Blackboxes
        #################################
        move_to_elevator.blackbox_level = py_trees.common.BlackBoxLevel.DETAIL
        ride.blackbox_level = py_trees.common.BlackBoxLevel.DETAIL

        #################################
        # Graph
        #################################

        children = []
        children.append(move_to_elevator)
        children.append(ride)
        children += teleport_and_initialise_children
        return children


class PartialAssistedElevators(Elevators):

    def __init__(self, name="Partial Assisted Elevators", origin=None, elevator=None, destination=None, elf_initialisation_type=elf.InitialisationType.TELEOP):
        """
        :param str name: behaviour name
        :param str origin: semantic world (unique_name) that it is coming from.
        :param str destination: semantic world (unique_name) that it is going to (another floor).
        :param gopher_semantic_msgs.Elevator elevator: the bridge between worlds
        """
        super(PartialAssistedElevators, self).__init__(name)
        if not origin or not elevator or not destination:
            (origin, elevator, destination) = get_default_args_from_semantics()
        self._origin = origin
        self._destination = destination
        self._elevator = elevator
        self._elevator_level_origin = get_elevator_level(elevator, origin)
        self._elevator_level_destination = get_elevator_level(elevator, destination)
        self._elf_initialisation_type = elf_initialisation_type
        self._gopher = gopher_configuration.Configuration(fallback_to_defaults=True)
        self._ride_uuid = uuid.uuid4().time_low

        ###########################################
        # Tree
        ###########################################
        # Assume all things here...as both semantics and planner should already validate everything.
        # use the add_child function to make sure that the parents are correctly initialised
        map(self.add_child, self._generate_elevator_children(self._gopher,
                                                             self._elevator.unique_name,
                                                             self._elevator_level_origin,
                                                             self._elevator_level_destination,
                                                             self._elf_initialisation_type))

    def _generate_elevator_children(self,
                                    gopher_configuration,
                                    elevator_name,
                                    elevator_level_origin,
                                    elevator_level_destination,
                                    elf_initialisation_type):
        """
        :param gopher_configuration.Configuration gopher_configuration:
        :param str elevator_name: unique name of the elevator
        :param gopher_semantic_msgs.ElevatorLevel elevator_level_origin:
        :param gopher_semantic_msgs.ElevatorLevel elevator_level_destination:
        """
        move_to_elevator = create_move_to_elevator_subtree(elevator_name, elevator_level_origin.entry)

        #################################
        # Pickup and Board
        #################################
        pickup = py_trees.composites.Parallel(name="Pickup",
                                              policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        call_elevator_to_floor = py_trees.composites.Sequence("Call Elevator")
        request_elevator_ride = elevator_operator_client.RequestElevatorRide("Request Ride",
                                                                             self._ride_uuid,
                                                                             elevator_level_origin.floor,
                                                                             elevator_level_destination.floor)
        expected_result_boarding = elevator_interactions_msgs.ElevatorOperatorStatus.WAITING_FOR_BOARDING
        wait_for_arrival_pickup = elevator_operator_client.RequestElevatorStatusUpdate("Wait for Arrival",
                                                                                       self._ride_uuid,
                                                                                       expected_result_boarding)
        honk_pickup = interactions.Articulate("Honk", gopher_configuration.sounds.honk)
        signal_wait_for_pickup = interactions.Notification(
            name="Flash Holding",
            led_pattern=gopher_configuration.led_patterns.holding,
            button_confirm=gopher_std_msgs.Notification.RETAIN_PREVIOUS,
            button_cancel=gopher_std_msgs.Notification.RETAIN_PREVIOUS,
            message="Waiting for the elevator to pick me up.")

        # board elevator
        board = interactions.flash_and_wait_for_go_button(
            name="Board",
            notification_behaviour_name="Flash for Input",
            led_pattern=gopher_configuration.led_patterns.humans_give_me_input,
            message="Waiting for boarding confirmation from human."
        )

        #################################
        # Ride
        #################################
        ride = py_trees.composites.Parallel(name="Ride",
                                            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        enjoy_ride = py_trees.composites.Sequence("Transit")
        passenger_status_boarded = elevator_interactions_msgs.PassengerStatus.BOARDED
        confirm_boarding = elevator_operator_client.PassengerStatusUpdate("Inform Operator\n'Ive Boarded'",
                                                                          self._ride_uuid,
                                                                          passenger_status_boarded)
        expected_result_boarding = elevator_interactions_msgs.ElevatorOperatorStatus.WAITING_FOR_DISEMBARKMENT
        wait_for_arrival_dropoff = elevator_operator_client.RequestElevatorStatusUpdate("Wait for Operator\n'Arrived'",
                                                                                        self._ride_uuid,
                                                                                        expected_result_boarding)
        honk_dropoff = interactions.Articulate("Honk", gopher_configuration.sounds.honk)
        signal_wait_for_dropoff = interactions.Notification(
            name="Flash Holding",
            led_pattern=gopher_configuration.led_patterns.holding,
            button_confirm=gopher_std_msgs.Notification.RETAIN_PREVIOUS,
            button_cancel=gopher_std_msgs.Notification.RETAIN_PREVIOUS,
            message="Waiting for the elevator to drop me off.")

        #################################
        # Disembark & Release
        #################################
        disembarkment = interactions.flash_and_wait_for_go_button(
            name="Disembark",
            notification_behaviour_name="Flash for Input",
            led_pattern=gopher_configuration.led_patterns.humans_give_me_input,
            message="Waiting for disembarkment confirmation from human."
        )
        release = py_trees.composites.Parallel(name="Release",
                                               policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        release_elevator = py_trees.composites.Sequence("Release Elevator")
        passenger_status_disembarked = elevator_interactions_msgs.PassengerStatus.DISEMBARKED
        confirm_disembarkment = elevator_operator_client.PassengerStatusUpdate("Inform Operator\n'Disembarked'",
                                                                               self._ride_uuid,
                                                                               passenger_status_disembarked)
        honk_disembarkment = interactions.Articulate("Honk", gopher_configuration.sounds.honk)
        signal_releasing_elevator = interactions.Notification(
            name="Flash Holding",
            led_pattern=gopher_configuration.led_patterns.holding,
            button_confirm=gopher_std_msgs.Notification.RETAIN_PREVIOUS,
            button_cancel=gopher_std_msgs.Notification.RETAIN_PREVIOUS,
            message="Releasing elevator.")

        #################################
        # Teleport and Initialise
        #################################
        try:
            teleport_and_initialise_children = create_teleport_and_initialise_children(
                elevator_name=elevator_name,
                elevator_level_origin=elevator_level_origin,
                elevator_level_destination=elevator_level_destination,
                elf_initialisation_type=elf_initialisation_type)
        except ValueError as e:
            raise e

        #################################
        # Blackboxes
        #################################
        move_to_elevator.blackbox_level = py_trees.common.BlackBoxLevel.DETAIL
        pickup.blackbox_level = py_trees.common.BlackBoxLevel.DETAIL
        board.blackbox_level = py_trees.common.BlackBoxLevel.DETAIL
        ride.blackbox_level = py_trees.common.BlackBoxLevel.DETAIL
        disembarkment.blackbox_level = py_trees.common.BlackBoxLevel.DETAIL
        release.blackbox_level = py_trees.common.BlackBoxLevel.DETAIL

        #################################
        # Graph
        #################################

        children = []
        children.append(move_to_elevator)
        children.append(pickup)
        pickup.add_child(call_elevator_to_floor)
        call_elevator_to_floor.add_children((request_elevator_ride, wait_for_arrival_pickup, honk_pickup))
        pickup.add_child(signal_wait_for_pickup)
        children.append(board)
        children.append(ride)
        ride.add_child(enjoy_ride)
        enjoy_ride.add_children((confirm_boarding, wait_for_arrival_dropoff, honk_dropoff))
        ride.add_child(signal_wait_for_dropoff)
        children.append(disembarkment)
        children.append(release)
        release.add_child(release_elevator)
        release_elevator.add_children((confirm_disembarkment, honk_disembarkment))
        release.add_child(signal_releasing_elevator)
        children += teleport_and_initialise_children

        return children

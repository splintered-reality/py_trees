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
import gopher_configuration
import gopher_navi_msgs.msg as gopher_navi_msgs
import py_trees
import gopher_std_msgs.msg as gopher_std_msgs
import operator
import rospy
import std_msgs.msg as std_msgs
import uuid

from . import elevator_operator_client
from . import elf
from . import interactions
from . import navigation


##############################################################################
# Helpers
##############################################################################

def get_elevator_level(elevator, world):
    for level in elevator.levels:
        if level.world == world:
            return level
    return None

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
        self.elevator_level_origin = get_elevator_level(elevator, origin)
        self.elevator_level_destination = get_elevator_level(elevator, destination)
        self.gopher = gopher_configuration.Configuration()
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
        move_to_elevator = navigation.MoveIt("To Elevator '%s'" % elevator_name, elevator_level_origin.entry)
        goal_finishing = navigation.GoalFinishing(name="Finish '%s'" % elevator_name, goal_pose=elevator_level_origin.entry)
        honk = interactions.Articulate("Honk", gopher_configuration.sounds.honk)

        travelling = py_trees.composites.Parallel(
            name="Travelling",
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE
        )

        flash_leds_travelling = interactions.Notification(
            name="Flash for Input",
            led_pattern=gopher_configuration.led_patterns.humans_give_me_input,
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


class PartialAssistedElevators(Elevators):

    def __init__(self, name, origin, elevator, destination):
        """
        :param str name: behaviour name
        :param str origin: semantic world (unique_name) that it is coming from.
        :param str destination: semantic world (unique_name) that it is going to (another floor).
        :param gopher_semantic_msgs.Elevator elevator: the bridge between worlds
        """
        super(PartialAssistedElevators, self).__init__(name)
        self._origin = origin
        self._destination = destination
        self._elevator = elevator
        self._elevator_level_origin = get_elevator_level(elevator, origin)
        self._elevator_level_destination = get_elevator_level(elevator, destination)
        self._config = gopher_configuration.Configuration()
        self._ride_uuid = uuid.uuid4().time_low

        ###########################################
        # Tree
        ###########################################
        # Assume all things here...as both semantics and planner should already validate everything.
        # use the add_child function to make sure that the parents are correctly initialised
        map(self.add_child, self._generate_elevator_children(self._config,
                                                             self._elevator.unique_name,
                                                             self._elevator_level_origin,
                                                             self._elevator_level_destination))

    def _generate_elevator_children(self,
                                    gopher_configuration,
                                    elevator_name,
                                    elevator_level_origin,
                                    elevator_level_destination):
        """
        :param gopher_configuration.Configuration gopher_configuration:
        :param str elevator_name: unique name of the elevator
        :param gopher_semantic_msgs.ElevatorLevel elevator_level_origin:
        :param gopher_semantic_msgs.ElevatorLevel elevator_level_destination:
        """
        children = []

        # move to elevator
        move_to_elevator = py_trees.composites.Sequence("Move to Elevator")
        navigate_to_elevator = navigation.MoveIt("Navigate to Elevator", elevator_level_origin.entry)
        honk_at_elevator = interactions.Articulate("Honk at Elevator", gopher_configuration.sounds.honk)
        move_to_elevator.add_children((navigate_to_elevator, honk_at_elevator))
        children.append(move_to_elevator)

        # Wait for pick-up
        wait_for_pickup = py_trees.composites.Parallel(name="Wait for Pick-up",
                                                       policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        call_elevator_to_floor = py_trees.composites.Sequence("Call Elevator to Pick-up Floor")
        request_elevator_ride = elevator_operator_client.RequestElevatorRide("Request Elevator Ride",
                                                                             self._ride_uuid,
                                                                             elevator_level_origin.floor,
                                                                             elevator_level_destination.floor)
        expected_result_boarding = elevator_interactions_msgs.ElevatorOperatorStatus.WAITING_FOR_BOARDING
        wait_for_arrival_pickup = elevator_operator_client.RequestElevatorStatusUpdate("Wait for Arrival at Pick-up",
                                                                                       self._ride_uuid,
                                                                                       expected_result_boarding)
        honk_pickup = interactions.Articulate("Honk at Pickup", gopher_configuration.sounds.honk)
        call_elevator_to_floor.add_children((request_elevator_ride, wait_for_arrival_pickup, honk_pickup))
        wait_for_pickup.add_child(call_elevator_to_floor)
        signal_wait_for_pickup = interactions.Notification(
            name="Signal Waiting for Pick-up",
            led_pattern=gopher_configuration.led_patterns.holding,
            button_confirm=gopher_std_msgs.Notification.RETAIN_PREVIOUS,
            button_cancel=gopher_std_msgs.Notification.RETAIN_PREVIOUS,
            cancel_on_stop=True,
            message="Waiting for the elevator to pick me up.")
        wait_for_pickup.add_child(signal_wait_for_pickup)
        children.append(wait_for_pickup)

        # board elevator
        boarding = py_trees.composites.Parallel(name="Elevator Boarding",
                                                policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        wait_for_boarding_confirmation = \
            py_trees.subscribers.WaitForSubscriberData(name="Wait for Boarding Confirmation",
                                                       topic_name=gopher_configuration.buttons.go,
                                                       topic_type=std_msgs.Empty)
        signal_waiting_boarding = interactions.Notification(
            name="Signal Waiting for Boarding",
            led_pattern=gopher_configuration.led_patterns.humans_give_me_input,
            button_confirm=gopher_std_msgs.Notification.BUTTON_ON,
            button_cancel=gopher_std_msgs.Notification.RETAIN_PREVIOUS,
            cancel_on_stop=True,
            message="Waiting for boarding confirmation from human.")
        boarding.add_children((wait_for_boarding_confirmation, signal_waiting_boarding))
        children.append(boarding)

        # ride elevator
        ride = py_trees.composites.Parallel(name="Elevator Ride",
                                            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        enjoy_ride = py_trees.composites.Sequence("Enjoy the Ride")
        passenger_status_boarded = elevator_interactions_msgs.PassengerStatus.BOARDED
        confirm_boarding = elevator_operator_client.PassengerStatusUpdate("Confirm Boarding",
                                                                          self._ride_uuid,
                                                                          passenger_status_boarded)
        expected_result_boarding = elevator_interactions_msgs.ElevatorOperatorStatus.WAITING_FOR_DISEMBARKMENT
        wait_for_arrival_dropoff = elevator_operator_client.RequestElevatorStatusUpdate("Wait for Arrival at Drop-off",
                                                                                        self._ride_uuid,
                                                                                        expected_result_boarding)
        honk_dropoff = interactions.Articulate("Honk at Dropoff", gopher_configuration.sounds.honk)
        enjoy_ride.add_children((confirm_boarding, wait_for_arrival_dropoff, honk_dropoff))
        ride.add_child(enjoy_ride)
        signal_wait_for_dropoff = interactions.Notification(
            name="Signal Waiting for Drop-off",
            led_pattern=gopher_configuration.led_patterns.holding,
            button_confirm=gopher_std_msgs.Notification.RETAIN_PREVIOUS,
            button_cancel=gopher_std_msgs.Notification.RETAIN_PREVIOUS,
            cancel_on_stop=True,
            message="Waiting for the elevator to drop me off.")
        ride.add_child(signal_wait_for_dropoff)
        children.append(ride)

        # disembark elevator
        disembarkment = py_trees.composites.Parallel(name="Elevator Disembarkment",
                                                     policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        wait_for_disembarkment_confirmation = \
            py_trees.subscribers.WaitForSubscriberData(name="Wait for Disembarkment Confirmation",
                                                       topic_name=gopher_configuration.buttons.go,
                                                       topic_type=std_msgs.Empty)
        signal_waiting_disembarkment = interactions.Notification(
            name="Signal Waiting for Disembarkment",
            led_pattern=gopher_configuration.led_patterns.humans_give_me_input,
            button_confirm=gopher_std_msgs.Notification.BUTTON_ON,
            button_cancel=gopher_std_msgs.Notification.RETAIN_PREVIOUS,
            cancel_on_stop=True,
            message="Waiting for disembarkment confirmation from human.")
        disembarkment.add_children((wait_for_disembarkment_confirmation, signal_waiting_disembarkment))
        children.append(disembarkment)

        # release elevator
        release = py_trees.composites.Parallel(name="Elevator Release",
                                               policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        release_elevator = py_trees.composites.Sequence("Release Elevator")
        passenger_status_disembarked = elevator_interactions_msgs.PassengerStatus.DISEMBARKED
        confirm_disembarkment = elevator_operator_client.PassengerStatusUpdate("Confirm Disembarkment",
                                                                               self._ride_uuid,
                                                                               passenger_status_disembarked)
        honk_disembarkment = interactions.Articulate("Honk at Disembarkment", gopher_configuration.sounds.honk)
        release_elevator.add_children((confirm_disembarkment, honk_disembarkment))
        release.add_child(release_elevator)
        signal_releasing_elevator = interactions.Notification(
            name="Signal Releasing Elevator",
            led_pattern=gopher_configuration.led_patterns.holding,
            button_confirm=gopher_std_msgs.Notification.RETAIN_PREVIOUS,
            button_cancel=gopher_std_msgs.Notification.RETAIN_PREVIOUS,
            cancel_on_stop=True,
            message="Releasing elevator.")
        release.add_child(signal_releasing_elevator)
        children.append(release)

        return children

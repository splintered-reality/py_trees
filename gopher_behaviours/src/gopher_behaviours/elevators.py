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
import gopher_semantics
import os
import py_trees

from . import interactions
from . import moveit
from . import navigation

from .time import Pause

##############################################################################
# Behaviours
##############################################################################


class ElevatorFailure(py_trees.Sequence):
    def __init__(self, name):
        """
        WARNING: this is not a very complete recovery behaviour. It expects the
        user to go all the way back to homebase and press the button. But doesn't
        inform him exactly that he should do this, nor does it handle
        map switching should it be required.

        :param str name: behaviour name
        """
        super(ElevatorFailure, self).__init__(name)
        self.gopher = gopher_configuration.Configuration()
        flash_leds = interactions.FlashLEDs("Warning", led_pattern=self.gopher.led_patterns.humans_i_need_help)
        wait_for_button = interactions.WaitForButton("go")
        flash_leds.add_child(wait_for_button)
        self.add_child(flash_leds)
        # self.add_child(something else?)


class HumanAssistedElevators(py_trees.Selector):

    def __init__(self, name, origin, destination, elevator):
        """
        :param str name: behaviour name
        :param gopher_semantic_msgs.Location origin: semantic location for where the robot is starting from.
        :param gopher_semantic_msgs.Location destination: semantic location for where it is going to (another floor).
        :param gopher_semantic_msgs.Elevator elevator: semantic elevator that makes the bridge
        """
        super(HumanAssistedElevators, self).__init__(name)
        self.origin = origin
        self.destination = destination
        self.elevator = elevator

        ###########################################
        # Load goodies from the rosparam server
        ###########################################
        self.gopher = gopher_configuration.Configuration()
        self.semantics = gopher_semantics.Semantics(self.gopher.namespaces.semantics)

        ###########################################
        # Checks
        ###########################################
        # Can assume all things here...as both semantics and planner should already have validated everything.
        # - e.g. that semantics has already validated that origin.world and destination.world are in the semantics database
        # - e.g. that planner   has already validated that origin.world and destination.world are different
        # - e.g. that planner   has already validated that they are reachable by an elevator

        elevator_sequence = self._generate_elevator_sequence()
        failure_sequence = ElevatorFailure("Failure")
        self.add_child(elevator_sequence)
        self.add_child(failure_sequence)

        def _generate_elevator_sequence():
            elevator_sequence = py_trees.Sequence("Elevator Run")
            elevator_level_origin = self.semantics.elevators.find_level_on_elevator(elevator.unique_name, origin.world)
            elevator_level_destination = self.semantics.elevators.find_level_on_elevator(elevator.unique_name, destination.world)

            # DJS : might need a delivery free MoveToGoal here
            move_to_elevator = moveit.MoveToGoal("To Elevator", elevator_level_origin.entry)
            honk = interactions.Articulate("Honk", self.gopher.sounds.honk)

            flash_leds_travelling = interactions.FlashLEDs("Travelling", led_pattern=self.gopher.led_patterns.humans_give_me_input)
            flash_leds_travelling.add_child(interactions.WaitForButton("Wait for Button", self.gopher.buttons.go))

            # TODO - get the map directory variable
            map_filename = os.path.join('somewhere', destination.world + ".dslam")
            switch_maps = navigation.SwitchMap("Switch Map (%s)" % map_filename, self.gopher.topics.switch_map)

            flash_leds_init = interactions.FlashLEDs("Initialising", led_pattern=self.gopher.led_patterns.holding)
            flash_leds_init.add_child(navigation.InitPose("Init Pose", elevator_level_destination.exit, self.gopher.topics.initial_pose))
            flash_leds_init.add_child(Pause("Wait For Init", 2.0))
            flash_leds_init.add_child(navigation.ClearCostmaps("ClearCostmaps"))

            elevator_sequence.add_child(move_to_elevator)
            elevator_sequence.add_child(honk)
            elevator_sequence.add_child(flash_leds_travelling)
            elevator_sequence.add_child(switch_maps)
            elevator_sequence.add_child(flash_leds_init)

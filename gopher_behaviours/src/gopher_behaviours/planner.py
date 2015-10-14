#!/usr/bin/env python

##############################################################################
# Imports
##############################################################################

import gopher_configuration
import gopher_semantics
import gopher_semantic_msgs.msg as gopher_semantic_msgs
import delivery
import moveit
import rocon_python_utils

from . import elevators

##############################################################################
# Implementation
##############################################################################


def find_topological_path(world, locations, semantics):
    """
    This is a very dumb pathfinder. It assumes all locations on
    a single world are connectable (i.e. doesn't find routes) and
    chooses the first possible connection between worlds (elevators)
    that it finds.

    No graph theory here yet!

    If no path can be found, it just returns the empty list.

    .. todo:: a more informative return, with a message saying which connection couldn't be made.

    :param [str] locations: list of semantic location unique names
    :param gopher_semantics.Semantics semantics: a semantics database
    :param str world: the unique name of the world we are starting from
    :return: a topological path of locations and elevators
    :rtype: list of gopher_semantic_msgs.Location and gopher_semantic_msgs.Elevator objects
    """
    last = None
    topological_path = []
    for location in locations:
        current = semantics.locations[location]
        previous_world = world if last is None else last.world
        if current.world != previous_world:
            connecting_elevators = semantics.elevators.find_connecting_elevators(previous_world, current.world)
            if not connecting_elevators:
                return []
            # just choose the first.
            connecting_elevator = connecting_elevators[0]
            topological_path.append(connecting_elevator)
        topological_path.append(current)
        last = current
    return topological_path


class Planner():

    def __init__(self, auto_go, disable_begin_end_behaviours=False):
        self.current_location = None
        self.disable_be_beh = disable_begin_end_behaviours
        self.auto_go = auto_go
        self.gopher = gopher_configuration.Configuration()
        self.semantics = gopher_semantics.Semantics(self.gopher.namespaces.semantics)

    def create_tree(self, current_world, locations, undock=True):
        """
        Find the semantic locations corresponding to the incoming string location identifier and
        create the appropriate behaviours.

        :param str current_world: unique_world name used to decide if we need to go straight to an elevator or not
        :param: str list of location unique names given to us by the delivery goal.

        .. todo::

           Clean up the key error handling
        """
        ########################################
        # Topological Path
        ########################################
        # TODO check all locations are in semantics, provide meaningful error message otherwise
        topological_path = find_topological_path(current_world, locations, self.semantics)
        # TODO check its not empty, error message if so

        ########################################
        # Tree
        ########################################
        children = []
        last_location = None
        for current_node, next_node in rocon_python_utils.iterables.lookahead(topological_path):
            previous_world = current_world if last_location is None else last_location.world
            if isinstance(current_node, gopher_semantic_msgs.Location):
                children.append(moveit.MoveToGoal(name=current_node.name, pose=current_node.pose))
                if next_node is not None:
                    children.append(
                        # spaces fubar the dot renderings....
                        delivery.Waiting(name="Waiting at " + current_node.name,
                                         location=current_node.unique_name,
                                         dont_wait_for_hoomans_flag=self.auto_go)
                    )
                last_location = current_node
            elif isinstance(current_node, gopher_semantic_msgs.Elevator):
                # topological path guarantees there is a next...
                elevator_subtree = elevators.HumanAssistedElevators("ElevatorRun", previous_world, current_node, next_node.world)
                children.append(elevator_subtree)

        # this can happen if none of the locations provided are in the semantic locations
        if not children:
            return None

        #################################################
        # Parking/Unparking or Docking/Undocking
        #################################################
        if undock and not self.disable_be_beh:
            children.insert(0, moveit.Starting("Starting"))

        # assume that we will go back to somewhere with a dock at the end of
        # each task, but only if the last location is homebase
        if locations[-1] == 'homebase' and not self.disable_be_beh:
            children.append(moveit.Finishing("Finishing"))

        return children

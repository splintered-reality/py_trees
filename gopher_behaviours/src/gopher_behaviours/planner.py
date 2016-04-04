#!/usr/bin/env python

##############################################################################
# Imports
##############################################################################

import gopher_behaviours
import gopher_configuration
import gopher_semantics
import gopher_semantic_msgs.msg as gopher_semantic_msgs
import delivery
import rocon_python_utils

from . import blackboard_handlers
from . import elevators
from . import interactions
from . import navigation

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

#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/yujinrobot/gopher_crazy_hospital/license/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
.. module:: delivery
   :platform: Unix
   :synopsis: Delivery behaviours

Gopher delivery behaviours! Are they good for anothing else?

----

"""

##############################################################################
# Imports
##############################################################################

from enum import Enum
import logging
import py_trees
import rospy
from . import moveit

##############################################################################
# Class
##############################################################################


class State(Enum):
    """ An enumerator representing the status of a behaviour """

    """Behaviour has no current goal."""
    IDLE = "IDLE"
    """Behaviour is waiting for a hooman interaction"""
    WAITING = "WAITING"
    """Behaviour is executing"""
    TRAVELLING = "TRAVELLING"
    """Behaviour is in an invalid state"""
    INVALID = "INVALID"


class Blackboard:
    """ Data store for the gopher delivery behaviours"""
    __shared_state = {
        'is_waiting': False,
        'traversed_locations': [],
        'remaining_locations': []
    }

    def __init__(self):
        self.__dict__ = self.__shared_state


class Waiting(py_trees.Behaviour):
    def __init__(self, name, location, dont_wait_for_hoomans_flag):
        super(Waiting, self).__init__(name)
        self.location = location
        self.blackboard = Blackboard()
        self.dont_wait_for_hoomans = dont_wait_for_hoomans_flag
        self.go_requested = False
        self.feedback_message = "hanging around at '%s' waiting for the lazy bastards to do something" % location

    def update(self):
        # check state to see if we just entered the behaviour
        if self.status == py_trees.Status.INVALID:
            self.blackboard.traversed_locations.append(self.blackboard.remaining_locations.pop(0))
            status = py_trees.Status.SUCCESS if self.dont_wait_for_hoomans else py_trees.Status.RUNNING
        if status == py_trees.Status.RUNNING and self.go_requested:
            status = py_trees.Status.SUCCESS
        return status


class GopherDeliveries(object):
    """
    :ivar root: root of the behaviour subtree for deliveries.
    """

    def __init__(self, name, locations=None):
        self.blackboard = Blackboard()
        self.blackboard.is_waiting = False
        self.incoming_goal = locations
        self.root = None
        self.state = State.IDLE
        self.locations = []
        self.has_a_new_goal = False
        self.dont_wait_for_hoomans = rospy.get_param("dont_wait_for_hoomans", True)
        self.feedback_message = ""

    def new_goal(self):
        """
        Callback for receipt of a new goal
        """
        # set self.incoming_goal = msg.semantic_locations (don't need a lock: http://effbot.org/pyfaq/what-kinds-of-global-value-mutation-are-thread-safe.htm)

    def pre_tick_update(self):
        if self.incoming_goal is not None and self.incoming_goal:
            children = []
            for location in self.incoming_goal:
                children.append(moveit.MoveToGoal(name=location.replace("_", " ").title()))
                children.append(Waiting(name="Waiting at " + location.replace("_", " ").title()), location, self.dont_wait_for_hoomans)
            children.append(moveit.GoHome(name="Heading Home"))
            self.root = py_trees.Sequence(name="Balli Balli Deliveries", children=children)
            self.blackboard.remaining_locations = self.incoming_goal
            self.locations = self.incoming_goal
            self.has_a_new_goal = True
        elif self.root is not None and self.root.status == Status.SUCCESS:
            self.root = None
            # new goal, even if it the none goal, this is the notification upstream that the tree needs to be swapped
            self.has_a_new_goal = True
        else:
            self.has_a_new_goal = False

    def post_tock_update(self):
        if self.root is None:
            self.state = State.IDLE
            self.feedback_message = "idling"
        elif isinstance(self.root.current_child, moveit.MoveToGoal):
            self.state = State.TRAVELLING
            self.feedback_message = "moving from '%s' to '%s'" % (self.blackboard.traversed_locations[-1], self.blackboard.remaining_locations[0])
        elif isinstance(self.root.current_child, Waiting):
            self.state = State.WAITING
            self.feedback_message = self.root.current_child.feedback_message

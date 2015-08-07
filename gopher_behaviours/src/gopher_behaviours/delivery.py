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

from enum import IntEnum
import collections
import os
import py_trees
import rocon_python_comms
import rocon_console.console as console
import rospkg
import rospy
from geometry_msgs.msg import Pose2D
import gopher_std_msgs.msg as gopher_std_msgs
import yaml
import std_msgs.msg as std_msgs
from . import moveit

##############################################################################
# Dummy Delivery Locations List (for testing)
##############################################################################


def desirable_destinations():
    """
    :return: list of gopher_std_msgs.Location objects
    """
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path("gopher_rocon_bootstrap")
    filename = os.path.join(pkg_path, "param", "semantic_locations", "desirable_destinations.yaml")
    desirables = []
    loaded_locations = collections.OrderedDict(sorted(yaml.load(open(filename))['semantic_locations'].items()))
    for key, value in loaded_locations.iteritems():
        location = gopher_std_msgs.Location()
        location.unique_name = key
        location.name = value['name']
        location.description = value['description']
        location.pose = Pose2D()
        location.pose.x = value['pose']['x']
        location.pose.y = value['pose']['y']
        location.pose.theta = value['pose']['x']
        location.keyframe_id = value['keyframe_id']
        desirables.append(location)
    return desirables

##############################################################################
# Machinery
##############################################################################


# this exactly matches the codes in gopher_std_msgs/DeliveryFeedback
class State(IntEnum):
    """ An enumerator representing the status of a delivery behaviour """

    """Behaviour has no current goal."""
    IDLE = 0
    """Behaviour is waiting for a hooman interaction"""
    WAITING = 1
    """Behaviour is executing"""
    TRAVELLING = 2
    """Behaviour is in an invalid state"""
    INVALID = 3


class Blackboard:
    """ Data store for the gopher delivery behaviours"""
    __shared_state = {
        'is_waiting': False,
        'traversed_locations': [],
        'remaining_locations': []
    }

    def __init__(self):
        self.__dict__ = self.__shared_state


##############################################################################
# Delivery Specific Behaviours
##############################################################################

class Waiting(py_trees.Behaviour):
    """
    Requires blackboard variables:

     - traversed_locations [list of strings]
     - remaining locations [list of strings]
    """
    def __init__(self, name, location, dont_wait_for_hoomans_flag):
        """
        :param str name: used for display
        :param str location: unique name of the location
        """
        super(Waiting, self).__init__(name)
        self.location = location
        self.blackboard = Blackboard()
        self.dont_wait_for_hoomans = dont_wait_for_hoomans_flag
        self.go_requested = False
        self.feedback_message = "hanging around at '%s' waiting for lazy bastards" % (location)

        # ros communications
        # could potentially have this in the waiting behaviour
        self._go_button_subscriber = rospy.Subscriber("delivery/go", std_msgs.Empty, self._go_button_callback)

    def initialise(self):
        """
        Called the first time the behaviour is triggered (i.e. when it transitions to the RUNNING state).

        .. seealso:: :py:meth:`Behaviour.initialise() <py_trees.behaviours.Behaviour.initialise>`
        """
        self.go_requested = False  # needs to be reset, just in case it's already true
        self.blackboard.traversed_locations.append(self.blackboard.remaining_locations.pop(0))

    def update(self):
        """
        Called by the behaviour :py:meth:`tick() <py_trees.behaviours.Behaviour.tick>` function.
        Does the real work...
        """
        status = py_trees.Status.SUCCESS if self.dont_wait_for_hoomans else py_trees.Status.RUNNING
        if status == py_trees.Status.RUNNING and self.go_requested:
            status = py_trees.Status.SUCCESS
        self.feedback_message = "remaining: %s" % self.blackboard.remaining_locations
        return status

    def _go_button_callback(self, unused_msg):
        self.go_requested = True if self.status == py_trees.Status.RUNNING else False


class GopherDeliveries(object):
    """
    Big picture view for bundling delivery behaviours.

    Can preload the delivery system with a set of semantic locations (in all their detail) or
    can reach out to a ros served set of semantic locations.

    Requires blackboard variables:

     - is_waiting [bool]

    :ivar root: root of the behaviour subtree for deliveries.
    :ivar blackboard:
    :ivar incoming_goal:
    :ivar locations:
    :ivar has_a_new_goal
    :ivar dont_wait_for_hoomans:
    :ivar feedback_message:
    :ivar old_goal_id:
    :ivar semantic_locations: pre-load the system with some semantic locations (list of gopher_std_msgs.Location)

    .. todo::

       going to be some broken logic here - if it's moving on its way home (i.e. running
       a different branch of the behaviour tree, then a new goal will still be accepted.
       We need some way of determining when this branch is active and not.
       Possibly have an idle state that replaces this root instead of just deleting it
       when it has no current goal? We can then check its status to see if it's the active
       branch or not...

       also need to mutex the self.incoming_goal variable
    """
    def __init__(self, name, semantic_locations=None):
        self.blackboard = Blackboard()
        self.blackboard.is_waiting = False
        self.root = None
        self.state = State.IDLE
        self.locations = []
        self.has_a_new_goal = False
        self.dont_wait_for_hoomans = rospy.get_param("~dont_wait_for_hoomans", False)
        self.feedback_message = ""
        self.old_goal_id = None
        self.semantic_locations = {}
        if semantic_locations is not None:
            self.incoming_goal = [location.unique_name for location in semantic_locations]
        else:
            self.incoming_goal = None
            semantic_locations = []  # fill this up from the map locations server
            try:
                response = rocon_python_comms.SubscriberProxy('/navi/semantic_locations', gopher_std_msgs.Locations)(rospy.Duration(5))  # TODO validate this is long enough for our purposes
                if response is not None:
                    rospy.loginfo("Gopher Deliveries : served semantic locations from the map locations server [%s]" % [location.unique_name for location in response.locations])
                    semantic_locations = response.locations
            except rospy.exceptions.ROSInterruptException:  # make sure to handle a Ros shutdown
                rospy.logwarn("Gopher Deliveries : ros shutdown(?) while attempting to connect to the map locations server")
                return
        for semantic_location in semantic_locations:
            self.semantic_locations[semantic_location.unique_name] = semantic_location
        rospy.logdebug("Gopher Deliveries : semantic locations served: \n%s" % self.semantic_locations)

    def set_goal(self, locations):
        """
        Callback for receipt of a new goal
        :param: list of semantic location strings (try to match these against the system served Map Locations list via the unique_name)
        :return: tuple of (gopher_std_msgs.DeliveryErrorCodes, string)
        """
        # don't need a lock: http://effbot.org/pyfaq/what-kinds-of-global-value-mutation-are-thread-safe.htm
        print("New goal: %s" % locations)
        if locations is None or not locations:
            return (gopher_std_msgs.DeliveryErrorCodes.GOAL_EMPTY_NOTHING_TO_DO, "goal empty, nothing to do.")
        if self.state == State.IDLE:
            self.incoming_goal = locations
            return (gopher_std_msgs.DeliveryErrorCodes.SUCCESS, "assigned new goal")
        elif self.state == State.WAITING:
            self.incoming_goal = locations
            return (gopher_std_msgs.DeliveryErrorCodes.SUCCESS, "pre-empting current goal")
        else:  # we're travelling between locations
            return (gopher_std_msgs.DeliveryErrorCodes.ALREADY_ASSIGNED_A_GOAL, "sorry, busy (already assigned a goal)")

    def pre_tick_update(self):
        """
        Check if we have a new goal, then assemble the sub-behaviours required to
        complete that goal. Finally kick out the current root and set the notification
        flag (self.has_a_new_goal) that this subtree has changed.
        """
        if self.incoming_goal is not None and self.incoming_goal:
            self.old_goal_id = self.root.id if self.root is not None else None
            children = self.locations_to_behaviours(self.incoming_goal)
            self.blackboard.traversed_locations = [] if not self.root else self.blackboard.traversed_locations
            self.root = py_trees.Sequence(name="Balli Balli Deliveries", children=children)
            self.blackboard.remaining_locations = self.incoming_goal
            self.locations = self.incoming_goal
            self.has_a_new_goal = True
            self.incoming_goal = None
        elif self.root is not None and self.root.status == py_trees.Status.SUCCESS:
            # last goal was achieved and no new goal, so swap this current subtree out
            self.old_goal_id = self.root.id if self.root is not None else None
            self.root = None
            self.has_a_new_goal = True
        else:
            self.has_a_new_goal = False

    def locations_to_behaviours(self, locations):
        """
        Find the semantic locations corresponding to the incoming string location identifier and
        create the appropariate behaviours.

        :param: string list of location unique names given to us by the delivery goal.
        """
        children = [] if self.root is not None else [moveit.UnDock("UnDock")]
        for location in locations[:-1]:
            semantic_location = self.semantic_locations[location]  # this is the full gopher_std_msgs.Location structure
            children.append(moveit.MoveToGoal(name=semantic_location.name, pose=semantic_location.pose))
            children.append(Waiting(name="Waiting at " + semantic_location.name,
                                    location=semantic_location.unique_name,
                                    dont_wait_for_hoomans_flag=self.dont_wait_for_hoomans)
                            )
        # special treatment for the last location
        semantic_location = self.semantic_locations[locations[-1]]  # this is the full gopher_std_msgs.Location structure
        children.append(moveit.MoveToGoal(name=semantic_location.name, pose=semantic_location.pose))
        #children.append(moveit.GoHome(name="Delivery Done"))

        return children

    def is_executing(self):
        """
        Is the robot currently mid-delivery?
        """
        return ((self.state == State.WAITING) or (self.state == State.TRAVELLING))

    def succeeded_on_last_tick(self):
        """
        Did the behaviour subtree succeed on the last tick?
        """
        if self.root is not None:
            return self.root.status == py_trees.Status.SUCCESS
        return False

    def post_tock_update(self):
        if self.root is not None and self.root.status == py_trees.Status.RUNNING:
            if isinstance(self.root.current_child(), moveit.MoveToGoal):
                self.state = State.TRAVELLING
                if self.blackboard.traversed_locations:
                    self.feedback_message = "moving from '%s' to '%s'" % (self.blackboard.traversed_locations[-1], self.blackboard.remaining_locations[0])
                else:
                    self.feedback_message = "moving to '%s'" % self.blackboard.remaining_locations[0]
            elif isinstance(self.root.current_child(), Waiting):
                self.state = State.WAITING
                self.feedback_message = self.root.current_child().feedback_message
        else:
            self.state = State.IDLE
            self.feedback_message = "idling"

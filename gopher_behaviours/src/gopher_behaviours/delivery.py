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
import rocon_console.console as console
import rospkg
import rospy
from geometry_msgs.msg import Pose2D
import gopher_std_msgs.msg as gopher_std_msgs
import yaml
import std_msgs.msg as std_msgs
from .blackboard import Blackboard
from . import moveit

##############################################################################
# Dummy Delivery Locations List (for testing)
##############################################################################


def desirable_destinations():
    """
    .. deprecated:: 0.1

       Moving to the new semantics yaml modules and gopher_semantic_msgs.

    :return: list of gopher_std_msgs.Location objects
    """
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path("gopher_semantics")
    filename = os.path.join(pkg_path, "param", "desirable_destinations.yaml")
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
        location.pose.theta = value['pose']['theta']
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
        self._notify_publisher = rospy.Publisher("~display_notification", gopher_std_msgs.Notification, queue_size=1)

        # for sending honk
        self._honk_publisher = None
        try:
            if rospy.get_param("~enable_honks"):
                honk_topic = rospy.get_param("~waiting_honk")
                self._honk_publisher = rospy.Publisher("/gopher/commands/sounds/" + honk_topic, std_msgs.Empty, queue_size=1)
        except KeyError:
            rospy.logwarn("Gopher Deliveries : Could not find param to initialise honks.")
            pass

    def initialise(self):
        """
        Called the first time the behaviour is triggered (i.e. when it transitions to the RUNNING state).

        .. seealso:: :py:meth:`Behaviour.initialise() <py_trees.behaviours.Behaviour.initialise>`
        """
        self.go_requested = False  # needs to be reset, just in case it's already true
        if self._honk_publisher:
            self._honk_publisher.publish(std_msgs.Empty())

    def update(self):
        """
        Called by the behaviour :py:meth:`tick() <py_trees.behaviours.Behaviour.tick>` function.
        Does the real work...
        """
        status = py_trees.Status.SUCCESS if self.dont_wait_for_hoomans else py_trees.Status.RUNNING
        if status == py_trees.Status.RUNNING and self.go_requested:
            status = py_trees.Status.SUCCESS
        else:
            self._notify_publisher.publish(gopher_std_msgs.Notification(buttons=gopher_std_msgs.Notification.BUTTON_GO))
        self.feedback_message = "remaining: %s" % self.blackboard.remaining_locations
        return status

    def _go_button_callback(self, unused_msg):
        self.go_requested = True if self.status == py_trees.Status.RUNNING else False
        self._notify_publisher.publish(gopher_std_msgs.Notification(buttons=gopher_std_msgs.Notification.BUTTONS_OFF))


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
    def __init__(self, name, planner, semantic_locations=None):
        self.blackboard = Blackboard()
        self.blackboard.is_waiting = False
        self.root = None
        self.state = State.IDLE
        self.locations = []
        self.has_a_new_goal = False
        self.feedback_message = ""
        self.old_goal_id = None
        self.planner = planner

        if semantic_locations is not None:
            self.incoming_goal = [location.unique_name for location in semantic_locations]
        else:
            self.incoming_goal = None

    def set_goal(self, locations):
        """
        Callback for receipt of a new goal
        :param: list of semantic location strings (try to match these against the system served Map Locations list via the unique_name)
        :return: tuple of (gopher_std_msgs.DeliveryErrorCodes, string)
        """
        # don't need a lock: http://effbot.org/pyfaq/what-kinds-of-global-value-mutation-are-thread-safe.htm
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

    def pre_tick_update(self, current_world):
        """
        Check if we have a new goal, then assemble the sub-behaviours required to
        complete that goal. Finally kick out the current root and set the notification
        flag (self.has_a_new_goal) that this subtree has changed.

        :param str current_world: update our knowledge of what the current world is.
        """
        if self.incoming_goal is not None and self.incoming_goal:
            # use the planner to initialise the behaviours that we are to follow
            # to get to the delivery location. If this is empty/None, then the
            # semantic locations provided were wrong.
            children = self.planner.create_tree(current_world, self.incoming_goal, undock=False if self.root else True)
            if not children:
                # TODO is this enough?
                rospy.logwarn("Gopher Deliveries : Received a goal, but none of the locations were valid.")
            else:
                self.old_goal_id = self.root.id if self.root is not None else None
                self.blackboard.traversed_locations = [] if not self.root else self.blackboard.traversed_locations
                self.root = py_trees.Sequence(name="Balli Balli Deliveries", children=children)
                self.blackboard.remaining_locations = self.incoming_goal
                self.locations = self.incoming_goal
                self.has_a_new_goal = True

            self.incoming_goal = None
        elif self.root is not None and self.root.status == py_trees.Status.SUCCESS:
            # if we succeeded, then we should be at the previously traversed
            # location (assuming that no task can occur in between locations.)
            self.planner.current_location = self.blackboard.traversed_locations[-1] if len(self.blackboard.traversed_locations) != 0 else None
            # last goal was achieved and no new goal, so swap this current subtree out
            self.old_goal_id = self.root.id if self.root is not None else None
            self.root = None
            self.has_a_new_goal = True
        else:
            self.has_a_new_goal = False

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

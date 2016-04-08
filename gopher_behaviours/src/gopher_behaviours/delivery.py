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

Gopher delivery behaviours! Are they good for anything else?

----

"""

##############################################################################
# Imports
##############################################################################

from enum import IntEnum
import collections
import os
import py_trees
# import rocon_console.console as console
import rospkg
import rospy
from geometry_msgs.msg import Pose2D
import gopher_std_msgs.msg as gopher_std_msgs
import gopher_std_msgs.srv as gopher_std_srvs
import gopher_delivery_msgs.msg as gopher_delivery_msgs
import yaml
import std_msgs.msg as std_msgs
import gopher_configuration
import unique_id
from py_trees.blackboard import Blackboard

from . import elevators
from . import recovery
from . import interactions
from . import navigation

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


# this exactly matches the codes in gopher_delivery_msgs/DeliveryFeedback
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


# this exactly matches the codes in gopher_delivery_msgs/DeliveryFeedback
class PreTickResult(IntEnum):
    """ An enumerator representing the status of a delivery behaviour """

    """Had an incoming goal and successfully setup a new delivery tree."""
    NEW_DELIVERY_TREE = 0
    """Had an incoming goal but failed to setup a new delivery tree."""
    NEW_DELIVERY_TREE_WITHERED_AND_DIED = 1
    """Delivery finished and delivery tree removed."""
    TREE_FELLED = 2
    """Nothing at all happened that is of interest."""
    BORING_NOTHING = 3

##############################################################################
# Delivery Specific Behaviours
##############################################################################


class Waiting(py_trees.Behaviour):
    """
    Requires blackboard variables:

     - traversed_locations [list of strings]
     - remaining locations [list of strings]
    """
    def __init__(self, name, location):
        """
        :param str name: used for display
        :param str location: unique name of the location
        """
        super(Waiting, self).__init__(name)
        self.config = gopher_configuration.Configuration()
        self.location = location
        self.blackboard = Blackboard()
        self.go_requested = False
        self.feedback_message = "hanging around at '%s' waiting for lazy bastards" % (location)
        self.notify_id = unique_id.toMsg(unique_id.fromRandom())

        # ros communications
        # could potentially have this in the waiting behaviour
        self._go_button_subscriber = rospy.Subscriber(self.config.buttons.go, std_msgs.Empty, self._go_button_callback)
        rospy.wait_for_service(self.config.services.notification)
        self._notify_srv = rospy.ServiceProxy(self.config.services.notification, gopher_std_srvs.Notify)

    def initialise(self):
        req = gopher_std_srvs.NotifyRequest()
        req.id = self.notify_id
        req.action = gopher_std_srvs.NotifyRequest.START
        req.duration = gopher_std_srvs.NotifyRequest.INDEFINITE
        req.notification = gopher_std_msgs.Notification(led_pattern=gopher_std_msgs.LEDStrip(led_strip_pattern=gopher_std_msgs.Notification.RETAIN_PREVIOUS),
                                                        button_confirm=gopher_std_msgs.Notification.BUTTON_ON,
                                                        button_cancel=gopher_std_msgs.Notification.RETAIN_PREVIOUS,
                                                        message="at location, waiting for button press")

        try:
            unused_response = self._notify_srv(req)
        except rospy.ServiceException as e:
            rospy.logwarn("SendNotification : Service failed to process notification request: {0}".format(str(e)))

    def update(self):
        """
        Called by the behaviour :py:meth:`tick() <py_trees.behaviours.Behaviour.tick>` function.
        Does the real work...
        """
        status = py_trees.Status.RUNNING
        if self.go_requested:
            status = py_trees.Status.SUCCESS

        self.feedback_message = "remaining: %s" % self.blackboard.remaining_locations
        return status

    def _go_button_callback(self, unused_msg):
        if self.status != py_trees.Status.RUNNING:
            return
        self.go_requested = True if self.status == py_trees.Status.RUNNING else False
        req = gopher_std_srvs.NotifyRequest()
        req.id = self.notify_id
        req.action = gopher_std_srvs.NotifyRequest.STOP
        req.notification = gopher_std_msgs.Notification(message="go button was pressed")

        try:
            unused_response = self._notify_srv(req)
        except rospy.ServiceException as e:
            rospy.logwarn("SendNotification : Service failed to process notification request: {0}".format(str(e)))


class GopherDeliveries(object):
    """
    Big picture view for bundling delivery behaviours.

    Can preload the delivery system with a set of semantic locations (in all their detail) or
    can reach out to a ros served set of semantic locations.

    :ivar root: root of the behaviour subtree for deliveries.
    :ivar blackboard:
    :ivar incoming_goal:
    :ivar locations:
    :ivar feedback_message:
    :ivar old_goal_id:

    Blackboard variables:

     - traversed_locations : [str]
     - remaining_locations : [str]

    .. todo::
       1. broken logic here
           If it's moving on its way home (i.e. running a different branch of the behaviour tree,\
           then a new goal will still be accepted. This should be handled higher up,\
           but should have a rejection here anyway just in case something behaves badly.

       2. Needs mutex
           To mutex the self.incoming_goal variable

    """
    def __init__(self, name, planner):
        self.blackboard = Blackboard()
        self.reset_blackboard_variables(traversed_locations=[], remaining_locations=[])
        self.config = gopher_configuration.Configuration()
        self.root = None
        self.state = State.IDLE
        self.locations = []
        self.feedback_message = ""
        self.old_goal_id = None
        self.planner = planner
        self.always_assume_initialised = False
        self.delivery_sequence = None
        self.incoming_goal = None

    def reset_blackboard_variables(self, traversed_locations=[], remaining_locations=[]):
        self.blackboard.traversed_locations = traversed_locations
        self.blackboard.remaining_locations = remaining_locations
        self.blackboard.cancel_requested = False

    def set_goal(self, locations, always_assume_initialised):
        """
        Callback for receipt of a new goal
        :param [str] locations: semantic locations (try to match these against the system served Map Locations list via the unique_name)
        :param bool always_assume_initialised: temporary flag for assuming the robot is initialised (right now we always assume uninitialised on the first run)
        :return: tuple of (gopher_delivery_msgs.DeliveryErrorCodes, string)
        """
        # don't need a lock: http://effbot.org/pyfaq/what-kinds-of-global-value-mutation-are-thread-safe.htm
        if locations is None or not locations:
            return (gopher_delivery_msgs.DeliveryErrorCodes.GOAL_EMPTY_NOTHING_TO_DO, "goal empty, nothing to do.")
        if self.state == State.IDLE:
            if self.planner.check_locations(locations):  # make sure locations are valid before returning success
                self.incoming_goal = locations
                self.always_assume_initialised = always_assume_initialised
                return (gopher_delivery_msgs.DeliveryErrorCodes.SUCCESS, "assigned new goal")
            else:
                return (gopher_delivery_msgs.DeliveryErrorCodes.FUBAR, "Received invalid locations")
        elif self.state == State.WAITING:
            self.incoming_goal = locations
            self.always_assume_initialised = always_assume_initialised
            return (gopher_delivery_msgs.DeliveryErrorCodes.SUCCESS, "pre-empting current goal")
        else:  # we're travelling between locations
            return (gopher_delivery_msgs.DeliveryErrorCodes.ALREADY_ASSIGNED_A_GOAL, "sorry, busy (already assigned a goal)")

    def cancel_goal(self):
        """
        This will set a blackboard variable that will trigger a delivery to cleanup itself. Usually
        this is triggered by a guard that implements a subtree recovery at a higher priority than the actual delivery.
        """
        self.blackboard.cancel_requested = True

    def pre_tick_update(self, current_world):
        """
        Check if we have a new goal, then assemble the sub-behaviours required to
        complete that goal. Finally kick out the current root if the last delivery
        succeeded.

        :param str current_world: update our knowledge of what the current world is.
        :returns: what happened...
        :rtype: PreTickResult
        """
        result = PreTickResult.BORING_NOTHING
        if self.incoming_goal is not None and self.incoming_goal:
            # use the planner to initialise the behaviours that we are to follow
            # to get to the delivery location. If this is empty/None, then the
            # semantic locations provided were wrong.
            include_parking_behaviours = True if not self.always_assume_initialised else False
            children = self.planner.create_tree(current_world,
                                                self.incoming_goal,
                                                include_parking_behaviours=include_parking_behaviours
                                                )
            if not children:
                # TODO is this enough?
                rospy.logwarn("Gopher Deliveries : Received a goal, but none of the locations were valid.")
            else:
                self.old_goal_id = self.root.id if self.root is not None else None
                # Blackboard
                self.reset_blackboard_variables(traversed_locations=[] if not self.root else self.blackboard.traversed_locations,
                                                remaining_locations=self.incoming_goal
                                                )
                # Assemble Behaviours
                self.cancel_requested = py_trees.CheckBlackboardVariable(name="Cancel Requested?",
                                                                         variable_name="cancel_requested",
                                                                         expected_value=True,
                                                                         invert=False
                                                                         )
                self.check_button_pressed = interactions.create_check_for_stop_button_press("Cancel Pressed?")
                self.cancel_required = py_trees.Selector(name="Cancellation Required?",
                                                         children=[self.check_button_pressed,
                                                                   self.cancel_requested]
                                                         )
                self.homebase_recovery = recovery.HomebaseRecovery(name="Delivery Cancelled")
                self.cancel_sequence = py_trees.Sequence(name="Cancellation",
                                                         children=[self.cancel_required,
                                                                   self.homebase_recovery
                                                                   ]
                                                         )
                # delivery sequence is oneshot - will only run once, retaining its succeeded or failed state
                self.delivery_sequence = py_trees.OneshotSequence(name="Balli Balli Deliveries",
                                                                  children=children)
                self.delivery_selector = py_trees.Selector(name="Deliver or recover",
                                                           children=[self.delivery_sequence,
                                                                     recovery.HomebaseRecovery("Delivery Failed")])
                self.root = py_trees.Selector(name="Deliver Unto Me",
                                              children=[self.cancel_sequence, self.delivery_selector])

                if self.root.setup(10):
                    self.locations = self.incoming_goal
                    result = PreTickResult.NEW_DELIVERY_TREE
                else:
                    rospy.logerr("Gopher Deliveries: failed to set up new delivery tree.")
                    if self.root is None:
                        rospy.logerr("Gopher Deliveries: shouldn't ever get here, but in case you do...call Dan")
                    result = PreTickResult.NEW_DELIVERY_TREE_WITHERED_AND_DIED
                    self.root = None
            self.incoming_goal = None

        elif self.root is not None and self.root.status == py_trees.Status.SUCCESS:
            # if we succeeded, then we should be at the previously traversed
            # location (assuming that no task can occur in between locations.)
            self.planner.current_location = self.blackboard.traversed_locations[-1] if len(self.blackboard.traversed_locations) != 0 else None
            # last goal was achieved and no new goal, so swap this current subtree out
            self.old_goal_id = self.root.id if self.root is not None else None
            self.root = None
            result = PreTickResult.TREE_FELLED
        return result

    def is_executing(self):
        """
        Is the robot currently mid-delivery?
        """
        return ((self.state == State.WAITING) or (self.state == State.TRAVELLING))

    def completed_on_last_tick(self):
        """
        Did the delivery tree finish on the last tick? This is a cover-all for
        finishing states - either delivered or recovered.
        """
        if self.root is not None:
            return self.root.status == py_trees.Status.SUCCESS
        return False

    def succeeded_on_last_tick(self):
        """
        Did the delivery subtree succeed on the last tick?
        """
        if self.root is not None:
            return self.delivery_selector.status == py_trees.Status.SUCCESS
        return False

    def recovered_on_last_tick(self):
        """
        Did the recovery subtree succeed on the last tick?
        """
        if self.root is not None:
            return self.cancel_sequence.status == py_trees.Status.SUCCESS
        return False

    def post_tock_update(self):
        """
        Be careful here, the logic gets tricky.
        """
        if self.root is not None and self.root.status == py_trees.Status.RUNNING:
            tip = self.root.tip()
            delivery_child = self.delivery_sequence.current_child
            if self.delivery_sequence.status == py_trees.Status.RUNNING:
                if any(map(lambda x: isinstance(delivery_child, x), [navigation.MoveIt, elevators.Elevators])):
                    self.state = State.TRAVELLING
                    if self.blackboard.traversed_locations:
                        self.feedback_message = "moving from '{0}' to '{1}'".format(self.blackboard.traversed_locations[-1], self.blackboard.remaining_locations[0])
                    else:
                        self.feedback_message = "moving to '{0}'".format(self.blackboard.remaining_locations[0])

                elif isinstance(tip, Waiting):
                    self.state = State.WAITING
                    self.feedback_message = tip.feedback_message
            else:  # we're in the homebase recovery behaviour
                self.state = State.WAITING  # don't allow it to be interrupted
                self.feedback_message = "delivery failed, waiting for human to teleop us back home before cancelling"
        else:
            self.state = State.IDLE
            self.feedback_message = "idling"

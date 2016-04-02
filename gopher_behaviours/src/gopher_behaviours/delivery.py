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
"""

##############################################################################
# Imports
##############################################################################

import enum
import gopher_configuration
import gopher_delivery_msgs.msg as gopher_delivery_msgs
import gopher_semantics
import gopher_semantic_msgs.msg as gopher_semantic_msgs
import gopher_std_msgs.msg as gopher_std_msgs
import gopher_std_msgs.srv as gopher_std_srvs
import py_trees
# import rocon_console.console as console
import rocon_python_utils
import rospy
import std_msgs.msg as std_msgs
import unique_id

from . import elevators
from . import recovery
from . import interactions
from . import navigation
from . import park
from . import planner
from . import unpark

##############################################################################
# Machinery
##############################################################################


class State(enum.IntEnum):
    """
    An enumerator representing the status of a delivery behaviour. It reflects
    what is currently in the delivery feedback message in a convenient enum
    format.
    """

    """Delivery has no current goal."""
    IDLE = gopher_delivery_msgs.DeliveryFeedback.IDLE
    """Delivery is waiting for a hooman interaction"""
    WAITING = gopher_delivery_msgs.DeliveryFeedback.WAITING
    """Delivery is executing"""
    TRAVELLING = gopher_delivery_msgs.DeliveryFeedback.TRAVELLING
    """Delivery is in an invalid state"""
    INVALID = gopher_delivery_msgs.DeliveryFeedback.INVALID
    """Delivery was cancelled"""
    CANCELLED = gopher_delivery_msgs.DeliveryFeedback.CANCELLED


class PreTickResult(enum.IntEnum):
    """ An enumerator representing the status of a delivery behaviour """

    """Had an incoming goal and successfully setup a new delivery tree."""
    NEW_DELIVERY_TREE = 0
    """Had an incoming goal but failed to setup a new delivery tree."""
    NEW_DELIVERY_TREE_WITHERED_AND_DIED = 1
    """Delivery finished and delivery tree removed."""
    TREE_FELLED = 2
    """Nothing at all happened that is of interest."""
    BORING_NOTHING = 3


def create_delivery_subtree(world, locations, include_parking_behaviours, express=False):
    """
    Build a new delivery subtree.

    :param str world: start from our current world
    :param [str] locations: semantic location string identifiers
    :param bool include_parking_behaviours: include the parking behaviours...or not
    :param bool express: dont wait when waiting
    """
    ########################
    # Moving Around
    ########################
    movin_around_children = create_locations_subtree(world, locations, express)
    if not movin_around_children:
        rospy.logerr("Gopher Deliveries : received a goal, but none of the locations were valid...we're FUBAR! (TODO need to handle this.")
        return (None, None)
    movin_around = py_trees.OneshotSequence(name="Balli Balli", children=movin_around_children)

    # TODO
    record_delivery_failure = py_trees.behaviours.Running(name="Record Delivery Failure")

    ########################
    # Recovery
    ########################
    recovery_required_by_cancel_request = py_trees.CheckBlackboardVariable(
        name="Cancel Requested?",
        variable_name="cancel_requested",
        expected_value=True,
        invert=False
    )
    recovery_required_by_button = interactions.create_check_for_stop_button_press("Stop Button?")
    # TODO
    recovery_required_by_delivery_failure = py_trees.behaviours.Failure("Delivery Failed?")

    recovery_check = py_trees.Selector(name="Needed?")
    recovery_check.add_child(recovery_required_by_button)
    recovery_check.add_child(recovery_required_by_cancel_request)
    recovery_check.add_child(recovery_required_by_delivery_failure)

    emergency_recovery = py_trees.Sequence(name="Emergency Recovery")
    homebase_recovery_children = recovery.create_homebase_recovery_children()

    root = py_trees.Sequence("Deliveries")
    delivery_manager = py_trees.Selector(name="Manager")

    ########################
    # Parking/Unparking
    ########################
    if include_parking_behaviours:
        unparking = unpark.UnPark("UnPark")
        parking = park.Park("Park")

    ########################
    # Building The Tree
    ########################
    if include_parking_behaviours:
        root.add_child(unparking)
    root.add_child(delivery_manager)
    if include_parking_behaviours:
        root.add_child(parking)
    delivery_manager.add_child(emergency_recovery)
    emergency_recovery.add_child(recovery_check)
    emergency_recovery.add_children(homebase_recovery_children)
    delivery_manager.add_child(movin_around)
    delivery_manager.add_child(record_delivery_failure)

    return (root, movin_around, emergency_recovery)


def create_locations_subtree(current_world, locations, express=False):
    """
    Find the semantic locations corresponding to the incoming string location identifier and
    create the appropriate behaviours.

    :param str current_world: unique_world name used to decide if we need to go straight to an elevator or not
    :param str[] locations: list of location unique names given to us by the delivery goal.
    :param bool express: don't wait at the waiting points

    .. todo::

       Clean up the key error handling
    """
    gopher = gopher_configuration.Configuration(fallback_to_defaults=True)
    semantics = gopher_semantics.Semantics(gopher.namespaces.semantics, fallback_to_defaults=True)
    ########################################
    # Topological Path
    ########################################
    # TODO check all locations are in semantics, provide meaningful error message otherwise
    topological_path = planner.find_topological_path(current_world, locations, semantics)
    # TODO check its not empty, error message if so

    ########################################
    # Tree
    ########################################
    children = []
    last_location = None
    for current_node, next_node in rocon_python_utils.iterables.lookahead(topological_path):
        previous_world = current_world if last_location is None else last_location.world
        if isinstance(current_node, gopher_semantic_msgs.Location):
            children.extend([
                navigation.MoveIt(
                    name="Go to " + current_node.name,
                    pose=current_node.pose
                ),
                navigation.GoalFinishing("Finish at " + current_node.name, current_node.pose),
                LocationTraversalHandler("Update Locations")]
            )
            if next_node is not None:
                children.extend([interactions.Articulate("Honk", gopher.sounds.honk)])
                if not express:
                    flash_leds_while_waiting = interactions.SendNotification(
                        "Flash - Waiting",
                        led_pattern=gopher.led_patterns.humans_give_me_input,
                        message="waiting for the user to tell me to proceed"
                    )
                    waiting = Waiting(name="Waiting at " + current_node.name, location=current_node.unique_name)
                    flash_leds_while_waiting.add_child(waiting)
                    children.extend([flash_leds_while_waiting])
            last_location = current_node
        elif isinstance(current_node, gopher_semantic_msgs.Elevator):
            # topological path guarantees there is a next...
            elevator_subtree = elevators.HumanAssistedElevators("ElevatorRun", previous_world, current_node, next_node.world)
            children.append(elevator_subtree)

    # this can happen if none of the locations provided are in the semantic locations
    if not children:
        return None

    children.append(interactions.Articulate("Done", gopher.sounds.done))
    return children

##############################################################################
# Delivery Specific Behaviours
##############################################################################


class LocationTraversalHandler(py_trees.Behaviour):
    """
    Blackboard Variables:

     - traversed_locations (rw) [str] : inserted from the front of the remaining locations
     - remaining_locations (rw) [str] : popped and pushed to the traversed locations
    """
    def __init__(self, name):
        super(LocationTraversalHandler, self).__init__(name)
        self.blackboard = py_trees.blackboard.Blackboard()

    def update(self):
        if not hasattr(self.blackboard, 'traversed_locations'):
            self.feedback_message = 'Blackboard did not have attribute traversed_locations. Location traversal failed.'
            return py_trees.Status.FAILURE
        elif not hasattr(self.blackboard, 'remaining_locations'):
            self.feedback_message = 'Blackboard did not have attribute remaining_locations. Location traversal failed.'
            return py_trees.Status.FAILURE
        else:
            try:
                self.blackboard.traversed_locations.append(self.blackboard.remaining_locations.pop(0))
            except IndexError:
                self.feedback_message = 'Traversed location list was empty, but pop was attempted'
                return py_trees.Status.FAILURE
            self.feedback_message = 'Traversal sucessful. New location is {0}'.format(self.blackboard.traversed_locations[-1])
            return py_trees.Status.SUCCESS


class Waiting(py_trees.Behaviour):
    """
    Blackboard Variables:

     - remaining_locations (r) [str] : used to provide a meaningful behaviour feedback message

    """
    def __init__(self, name, location):
        """
        :param str name: used for display
        :param str location: unique name of the location
        """
        super(Waiting, self).__init__(name)
        self.config = gopher_configuration.Configuration()
        self.location = location
        self.blackboard = py_trees.blackboard.Blackboard()
        self.go_requested = False
        self.feedback_message = "hanging around at '%s' waiting for lazy bastards" % (location)
        self.notify_id = unique_id.toMsg(unique_id.fromRandom())

    def setup(self, timeout):
        """Delayed ROS Setup"""
        self._go_button_subscriber = rospy.Subscriber(self.config.buttons.go, std_msgs.Empty, self._go_button_callback)
        try:
            rospy.wait_for_service(self.config.services.notification, timeout=timeout)
        except rospy.ROSException:  # timeout
            return False
        except rospy.ROSInterruptException:  # shutdown
            return False
        self._notify_srv = rospy.ServiceProxy(self.config.services.notification, gopher_std_srvs.Notify)
        return True

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
    def __init__(self, name):
        self.blackboard = py_trees.blackboard.Blackboard()
        self.reset_blackboard_variables(traversed_locations=[], remaining_locations=[])
        self.config = gopher_configuration.Configuration()
        self.root = None
        self.state = State.IDLE
        self.current_location = None
        self.locations = []
        self.feedback_message = ""
        self.old_goal_id = None
        self.include_parking_behaviours = False
        self.express = False
        self.delivery_locations_subtree = None
        self.emergency_recovery_subtree = None
        self.incoming_goal = None
        self.gopher = None
        self.semantics = None

    def setup(self, timeout):
        """
        Delayed ROS Setup
        """
        self.gopher = gopher_configuration.Configuration()
        self.semantics = gopher_semantics.Semantics(self.gopher.namespaces.semantics)

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
            # make sure locations are valid before returning success
            if all([location in self.semantics.locations for location in locations]):
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

            self.old_goal_id = self.root.id if self.root is not None else None
            self.reset_blackboard_variables(traversed_locations=[] if not self.root else self.blackboard.traversed_locations,
                                            remaining_locations=self.incoming_goal
                                            )
            (self.root, self.delivery_locations_subtree, self.emergency_recovery_subtree) = create_delivery_subtree(
                current_world,
                self.incoming_goal,
                self.include_parking_behaviours,
                self.express
            )

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
            self.current_location = self.blackboard.traversed_locations[-1] if len(self.blackboard.traversed_locations) != 0 else None
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
            return self.delivery_locations_subtree.status == py_trees.Status.SUCCESS
        return False

    def recovered_on_last_tick(self):
        """
        Did the recovery subtree succeed on the last tick?
        """
        if self.root is not None:
            return self.emergency_recovery_subtree.status == py_trees.Status.SUCCESS
        return False

    def post_tock_update(self):
        """
        Be careful here, the logic gets tricky.
        """
        if self.root is not None and self.root.status == py_trees.Status.RUNNING:
            tip = self.root.tip()
            delivery_child = self.delivery_locations_subtree.current_child
            if self.delivery_locations_subtree.status == py_trees.Status.RUNNING:
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

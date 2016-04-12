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
from . import interactions
from . import navigation
from . import park
from . import planner
from . import unpark

##############################################################################
# Machinery
##############################################################################


class State(enum.Enum):
    """
    An enumerator representing the status of a delivery behaviour. It reflects
    what is currently in the delivery feedback message in a convenient enum
    format.
    """

    """Ready to accept a goal."""
    IDLE = "IDLE"
    """Interrupted by a higher priority branch."""
    INTERRUPTED = "INTERRUPTED"
    """Unparking, signifies the start of a delivery"""
    UNPARKING = "UNPARKING"
    """On the delivery route"""
    EN_ROUTE = "EN_ROUTE"
    """Parking, signifying the end of a delivery run"""
    PARKING = "PARKING"
    """Cancelling, an automated run home upon request"""
    CANCELLING = "CANCELLING"
    """Reparking after a delivery has failed for whatever reason"""
    RECOVERING = "RECOVERING"


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


def create_delivery_subtree(world, locations, express=False):
    """
    Build a new delivery subtree.

    :param str world: start from our current world
    :param [str] locations: semantic location string identifiers
    :param bool express: dont wait when waiting
    """
    gopher = gopher_configuration.Configuration(fallback_to_defaults=True)

    ########################
    # Cancel Guard
    ########################
    cancelled_by_cancel_request = py_trees.blackboard.CheckBlackboardVariable(
        name="Cancel Requested?",
        variable_name="cancel_requested",
        expected_value=True
    )
    cancelled_by_button = interactions.create_check_for_stop_button_press("Stop Button?")

    is_cancelled = py_trees.meta.inverter(py_trees.Selector)(name="Cancelled?")

    ########################
    # Moving Around
    ########################
    en_route = py_trees.OneshotSequence(name="En Route")
    movin_around_children = create_locations_subtree(world, locations, express)

    if not movin_around_children:
        # how to get a warning back? also need to handle this shit
        return (None, None)

    ########################
    # Parking/Unparking
    ########################
    unparking = unpark.UnPark("UnPark")
    parking = park.Park("Park")
    recovering = py_trees.composites.Sequence(name="Recovering")
    repark = park.create_repark_subtree()

    ########################
    # Core
    ########################
    root = py_trees.composites.Selector(name="Deliver or Die")
    deliveries = py_trees.Sequence("Deliveries")
    todo_or_not = py_trees.composites.Selector(name="Do or be Cancelled?")
    delivery = py_trees.composites.Parallel(
        name="Delivery",
        policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL
    )

    ########################
    # Teleop Home on Cancel
    ########################
    cancelling = py_trees.composites.Sequence("Cancelled")
    teleop_to_homebase = py_trees.composites.Parallel(
        name="Teleop to Homebase",
        policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE
    )
    flash_i_need_help = interactions.Notification(
        name='Flash Help Me',
        message='waiting for button press to continue',
        led_pattern=gopher.led_patterns.humans_i_need_help,
        button_confirm=gopher_std_msgs.Notification.BUTTON_ON,
        button_cancel=gopher_std_msgs.Notification.RETAIN_PREVIOUS,
        cancel_on_stop=True,
        duration=gopher_std_srvs.NotifyRequest.INDEFINITE
    )
    wait_for_go_button_press = interactions.create_wait_for_go_button("Wait for Go Button")
    teleport = navigation.create_homebase_teleport()

    ########################
    # Delivery State
    ########################
    state_cancelling = py_trees.blackboard.SetBlackboardVariable(name="DS 'Cancelling'", variable_name="delivery_state", variable_value=State.CANCELLING)
    state_en_route = py_trees.blackboard.SetBlackboardVariable(name="DS 'En Route'", variable_name="delivery_state", variable_value=State.EN_ROUTE)
    state_unparking = py_trees.blackboard.SetBlackboardVariable(name="DS 'Unparking'", variable_name="delivery_state", variable_value=State.UNPARKING)
    state_parking = py_trees.blackboard.SetBlackboardVariable(name="DS 'Parking'", variable_name="delivery_state", variable_value=State.PARKING)
    state_recovering = py_trees.blackboard.SetBlackboardVariable(name="DS 'Recovering'", variable_name="delivery_state", variable_value=State.RECOVERING)

    ########################
    # Blackboxes
    ########################
    todo_or_not.blackbox_level = py_trees.common.BlackBoxLevel.BIG_PICTURE
    delivery.blackbox_level = py_trees.common.BlackBoxLevel.COMPONENT
    recovering.blackbox_level = py_trees.common.BlackBoxLevel.COMPONENT
    cancelling.blackbox_level = py_trees.common.BlackBoxLevel.COMPONENT
    teleop_to_homebase.blackbox_level = py_trees.common.BlackBoxLevel.DETAIL

    ########################
    # Graph
    ########################
    root.add_child(deliveries)
    deliveries.add_child(state_unparking)
    deliveries.add_child(unparking)
    deliveries.add_child(todo_or_not)
    todo_or_not.add_child(delivery)
    delivery.add_child(is_cancelled)
    is_cancelled.add_child(cancelled_by_button)
    is_cancelled.add_child(cancelled_by_cancel_request)
    delivery.add_child(en_route)
    en_route.add_child(state_en_route)
    en_route.add_children(movin_around_children)
    todo_or_not.add_child(cancelling)
    cancelling.add_child(state_cancelling)
    cancelling.add_child(teleop_to_homebase)
    teleop_to_homebase.add_child(flash_i_need_help)
    teleop_to_homebase.add_child(wait_for_go_button_press)
    cancelling.add_child(teleport)
    deliveries.add_child(state_parking)
    deliveries.add_child(parking)
    root.add_child(recovering)
    recovering.add_child(state_recovering)
    recovering.add_child(repark)

    return (root, deliveries, en_route, is_cancelled)


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
            go_to = py_trees.composites.Sequence(
                name="Go to %s" % current_node.name
            )
            move_to = navigation.MoveIt(
                name="Move To " + current_node.name,
                pose=current_node.pose
            )
            goal_finishing = navigation.GoalFinishing("Finish at %s" % current_node.name, current_node.pose)
            update_locations = LocationTraversalHandler("Update Locations")
            go_to.add_child(move_to)
            go_to.add_child(goal_finishing)
            go_to.add_child(update_locations)
            go_to.blackbox_level = py_trees.common.BlackBoxLevel.DETAIL
            waiting = None
            if next_node is not None:
                honk = interactions.Articulate("Honk", gopher.sounds.honk)
                go_to.add_child(honk)
                if not express:
                    waiting = py_trees.composites.Parallel(
                        name="Waiting",
                        policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE
                    )
                    flash_leds_while_waiting = interactions.Notification(
                        "Flash for Input",
                        led_pattern=gopher.led_patterns.humans_give_me_input,
                        message="waiting for the user to tell me to proceed"
                    )
                    waiting_at = Waiting(name="Waiting at %s" % current_node.name, location=current_node.unique_name)
                    waiting.blackbox_level = py_trees.common.BlackBoxLevel.DETAIL
                    waiting.add_child(flash_leds_while_waiting)
                    waiting.add_child(waiting_at)
            children.extend([go_to])
            if waiting is not None:
                children.extend([waiting])
            last_location = current_node
        elif isinstance(current_node, gopher_semantic_msgs.Elevator):
            # topological path guarantees there is a next...
            elevator_subtree = elevators.HumanAssistedElevators("Elevator to %s" % next_node.world, previous_world, current_node, next_node.world)
            elevator_subtree.blackbox_level = py_trees.common.BlackBoxLevel.DETAIL
            children.append(elevator_subtree)

    # this can happen if none of the locations provided are in the semantic locations
    if not children:
        return None

    children.append(interactions.Articulate("Groot 'Done'", gopher.sounds.done))
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
     - current_location    (w) [str] : update the corrent location

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
            rospy.logwarn("Notification : Service failed to process notification request: {0}".format(str(e)))

        self.blackboard.current_location = self.location

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
            rospy.logwarn("Notification : service failed to process notification request: {0}".format(str(e)))

##############################################################################
# Deliveries
##############################################################################


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

     * delivery_state      (rw) [str]   : behaviours in the subtree write the current state as they pass through
     * battery_low_warning (r)  [int]   : used to check if a goal should be accepted or not
     * traversed_locations (rw) [[str]] :
     * remaining_locations (rw) [[str]] :

    """
    #################################
    # Initialisation
    #################################
    def __init__(self, name):
        self.blackboard = py_trees.blackboard.Blackboard()
        self.init()
        self.gopher = None
        self.semantics = None
        self.ros_connected = False  # whether the underlying ros subsystem fully connected or not
        self.locations = []
        self.feedback_message = ""
        self.old_goal_id = None
        self.express = False
        self.incoming_goal = None

    def init(self):
        """
        Reusable initialisation for when we clear the delivery tree.
        """
        self.root = None
        self.deliveries_subtree = None
        self.en_route_subtree = None
        self.is_cancelled_subtree = None
        self.init_blackboard_variables(traversed_locations=[], remaining_locations=[])
        self.state = self.blackboard.delivery_state

    @property
    def state(self):
        return self.blackboard.delivery_state

    @state.setter
    def state(self, new_state):
        self.blackboard.delivery_state = new_state

    def setup(self, timeout):
        """
        Delayed ROS Setup, creates an example tree and tries to set that up. If this fails
        the delivery tree is considered dysfunctional.
        """
        self.gopher = gopher_configuration.Configuration()
        self.semantics = gopher_semantics.Semantics(self.gopher.namespaces.semantics)
        (deliveries_root, unused_deliveries, unused_en_route, unused_is_cancelled) = create_delivery_subtree(
            world="earth",
            locations=["beer_fridge", "ashokas_hell"],
            express=False
        )
        self.ros_connected = deliveries_root.setup(timeout)
        if not self.ros_connected:
            rospy.logerr("Deliveries : failed to setup with the underlying ros subsystem")

    def init_blackboard_variables(self, traversed_locations=[], remaining_locations=[]):
        self.blackboard.delivery_state = State.IDLE
        self.blackboard.traversed_locations = traversed_locations
        self.blackboard.remaining_locations = remaining_locations
        self.blackboard.cancel_requested = False

    #################################
    # Goal Management
    #################################

    def set_goal(self, locations):
        """
        Callback for receipt of a new goal
        :param [str] locations: semantic locations (try to match these against the system served Map Locations list via the unique_name)
        :return: tuple of (gopher_delivery_msgs.DeliveryErrorCodes, string)
        """
        # don't need a lock: http://effbot.org/pyfaq/what-kinds-of-global-value-mutation-are-thread-safe.htm
        if locations is None or not locations:
            return (gopher_delivery_msgs.DeliveryErrorCodes.REJECT_GOAL_EMPTY_GOAL_NOTHING_TO_DO, "goal empty, nothing to do.")
        if not self.ros_connected:
            return (gopher_delivery_msgs.DeliveryErrorCodes.REJECT_GOAL_SYSTEM_NOT_READY, "underlying ros subsystem not properly connected")
        if self.blackboard.battery_low_warning:  # TODO this probably needs to be smarter
            return (gopher_delivery_msgs.DeliveryErrorCodes.REJECT_GOAL_NOT_ENOUGH_JUICE, "not enough juice")
        if self.state == State.IDLE:
            # make sure locations are valid before returning success
            if all([location in self.semantics.locations for location in locations]):
                self.incoming_goal = locations
                return (gopher_delivery_msgs.DeliveryErrorCodes.SUCCESS, "assigned new goal")
            else:
                return (gopher_delivery_msgs.DeliveryErrorCodes.REJECT_GOAL_INVALID_LOCATION, "Received invalid locations")
        elif self.state == State.WAITING:
            self.incoming_goal = locations
            return (gopher_delivery_msgs.DeliveryErrorCodes.SUCCESS, "pre-empting current goal")
        elif self.state == State.BUSY:
            return (gopher_delivery_msgs.DeliveryErrorCodes.REJECT_GOAL_ROBOT_IS_BUSY, "sorry, busy looking for a robot girl")
        elif self.state == State.REINITIALISING:
            return (gopher_delivery_msgs.DeliveryErrorCodes.REJECT_GOAL_ALREADY_ASSIGNED, "sorry, busy (re-initialising mid-goal)")
        else:  # we're travelling between locations
            return (gopher_delivery_msgs.DeliveryErrorCodes.REJECT_GOAL_ALREADY_ASSIGNED, "sorry, busy (already assigned a goal)")

    def cancel_goal(self):
        """
        This will set a blackboard variable that will trigger a delivery to cleanup itself. Usually
        this is triggered by a guard that implements a subtree recovery at a higher priority than the actual delivery.
        """
        self.blackboard.cancel_requested = True

    #################################
    # Tick Tock
    #################################

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
            self.init_blackboard_variables(traversed_locations=[] if not self.root else self.blackboard.traversed_locations,
                                           remaining_locations=self.incoming_goal
                                           )
            (self.root, self.deliveries_subtree, self.en_route_subtree, self.is_cancelled_subtree) = create_delivery_subtree(
                current_world,
                self.incoming_goal,
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
                self.init()
            self.incoming_goal = None

        elif self.root is not None and self.root.status == py_trees.Status.SUCCESS:
            # last goal was achieved and no new goal, so swap this current subtree out
            self.old_goal_id = self.root.id if self.root is not None else None
            self.init()
            result = PreTickResult.TREE_FELLED
        return result

    def post_tock_update(self):
        """
        Check the current state and set a feedback message accordingly.
        """
        if self.root is None or self.root.status == py_trees.common.Status.SUCCESS:
            self.feedback_message = State.IDLE
            self.blackboard.delivery_state = State.IDLE
        elif self.blackboard.delivery_state == State.EN_ROUTE:
            tip = self.root.tip()
            delivery_child = self.en_route_subtree.current_child
            if self.en_route_subtree.status == py_trees.Status.RUNNING:
                if any(map(lambda x: isinstance(delivery_child, x), [navigation.MoveIt, elevators.Elevators])):
                    if self.blackboard.traversed_locations:
                        self.feedback_message = "moving from '{0}' to '{1}'".format(self.blackboard.traversed_locations[-1], self.blackboard.remaining_locations[0])
                    else:
                        self.feedback_message = "moving to '{0}'".format(self.blackboard.remaining_locations[0])
                elif isinstance(tip, Waiting):
                    self.feedback_message = tip.feedback_message
        else:
            self.feedback_message = str(self.blackboard.delivery_state).title()

    #################################
    # Introspection
    #################################

    def has_subtree(self):
        return self.root.status is not None

    def is_running(self):
        if self.root is not None:
            return self.root.status == py_trees.Status.RUNNING
        return False

    def is_interrupted(self):
        return self.root is not None and self.root.status == py_trees.Status.INVALID

    def is_finished(self):
        """
        Did the delivery tree finish on the last tick? Inbetween ticks this
        will only fire once since the tree will immediately be deleted next
        pre-tick.
        """
        if self.root is not None:
            return self.root.status == py_trees.Status.SUCCESS
        return False

    def is_running_but_cancelled(self):
        if self.root is not None and self.root.status != py_trees.Status.INVALID:
            return self.is_cancelled_subtree.status == py_trees.Status.FAILURE
        return False

    def is_running_and_en_route(self):
        if self.root is not None and self.en_route.status == py_trees.Status.RUNNING:
            return self.en_route_subtree.status != py_trees.Status.INVALID
        return False

    def is_running_but_failed(self):
        if self.root is not None and self.root.status != py_trees.Status.INVALID:
            return self.deliveries_subtree.status == py_trees.Status.FAILURE
        return False

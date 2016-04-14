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

import copy
import dynamic_reconfigure.server
import gopher_configuration
import gopher_delivery_msgs.msg as gopher_delivery_msgs
import gopher_delivery_msgs.srv as gopher_delivery_srvs
import gopher_semantics
import gopher_semantic_msgs.msg as gopher_semantic_msgs
import gopher_std_msgs.msg as gopher_std_msgs
import gopher_std_msgs.srv as gopher_std_srvs
import py_trees
import rocon_console.console as console
import rocon_python_comms
import rocon_python_utils
import rospy
import std_msgs.msg as std_msgs
import threading
import unique_id

from . import elevators
from . import elf
from . import interactions
from . import navigation
from . import park
from . import planner
from . import unpark

from gopher_behaviours.cfg import QuirkyDeliveriesConfig


##############################################################################
# Machinery
##############################################################################


class Parameters(object):
    """
    The variables of this class are default constructed from parameters on the
    ros parameter server. Each parameter is nested in the private namespace of
    the node which instantiates this class.

    :ivar express: whether deliveries should stop and wait for a button press at each location (or not)
    :vartype express: bool
    """
    def __init__(self):
        """
        Initial parameters are create to be sufficient for starting up a 'dummy'
        tree and calling :py::meth:`~gopher_behaviours.delivery.QuirkyDeliveries.setup` on that tree.
        """
        self.elf = elf.InitialisationType.TELEOP
        self.express = False
        self.parking = False

    def __str__(self):
        s = console.bold + "\nParameters:\n" + console.reset
        for key in sorted(self.__dict__):
            s += console.cyan + "    %s: " % key + console.yellow + "%s\n" % (self.__dict__[key] if self.__dict__[key] is not None else '-')
        s += console.reset
        return s


class Subtrees(object):
    def __init__(self):
        self.unparking = None
        self.parking = None
        self.recovering = None
        self.cancelling = None
        self.is_cancelled = None
        self.en_route = None


def create_delivery_subtree(world, locations, parameters=Parameters()):
    """
    Build a new delivery subtree.

    :param str world: start from our current world
    :param [str] locations: semantic location string identifiers
    :param Parameters parameters: list of parameters to use for configuration of the tree
    """
    subtrees = Subtrees()

    ########################
    # Cancel Guard
    ########################
    subtrees.cancelling = py_trees.composites.Sequence(name="Cancelling")
    subtrees.is_cancelled = py_trees.Selector(name="Is Cancelled?")
    cancelled_by_cancel_request = py_trees.blackboard.CheckBlackboardVariable(
        name="Cancel Requested?",
        variable_name="cancel_requested",
        expected_value=True
    )
    cancelled_by_button = py_trees.blackboard.CheckBlackboardVariable(
        name="Stop Button?",
        variable_name="event_stop_button",
        expected_value=True
    )
    teleop_and_teleport = navigation.create_teleop_homebase_teleport_subtree()

    ########################
    # Moving Around
    ########################
    subtrees.en_route = py_trees.OneshotSequence(name="En Route")
    movin_around_children = create_locations_subtree(world, locations, parameters.express)

    if not movin_around_children:
        # how to get a warning back? also need to handle this shit
        return (None, None)

    ########################
    # Parking/Unparking
    ########################
    subtrees.unparking = unpark.UnPark(name="UnPark", elf_type=parameters.elf)
    subtrees.parking = park.Park("Park")
    subtrees.recovering = py_trees.composites.Sequence(name="Recovering")
    repark = park.create_repark_subtree()

    ########################
    # Core
    ########################
    root = py_trees.composites.Selector(name="Deliver or Die")
    subtrees.deliveries = py_trees.OneshotSequence("Deliveries")
    todo_or_not = py_trees.composites.Selector(name="Do or be Cancelled?")

    ########################
    # Blackboxes
    ########################
    todo_or_not.blackbox_level = py_trees.common.BlackBoxLevel.BIG_PICTURE
    subtrees.is_cancelled.blackbox_level = py_trees.common.BlackBoxLevel.DETAIL
    subtrees.recovering.blackbox_level = py_trees.common.BlackBoxLevel.COMPONENT
    subtrees.cancelling.blackbox_level = py_trees.common.BlackBoxLevel.COMPONENT

    ########################
    # Graph
    ########################
    root.add_child(subtrees.deliveries)
    subtrees.deliveries.add_child(subtrees.unparking)
    subtrees.deliveries.add_child(todo_or_not)
    todo_or_not.add_child(subtrees.cancelling)
    subtrees.cancelling.add_child(subtrees.is_cancelled)
    subtrees.is_cancelled.add_child(cancelled_by_button)
    subtrees.is_cancelled.add_child(cancelled_by_cancel_request)
    subtrees.cancelling.add_child(teleop_and_teleport)

    todo_or_not.add_child(subtrees.en_route)
    subtrees.en_route.add_children(movin_around_children)
    subtrees.deliveries.add_child(subtrees.parking)
    root.add_child(subtrees.recovering)
    subtrees.recovering.add_child(repark)

    return (root, subtrees)


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
        self.incoming_goal = None
        self.current_world = None
        self.parameters = Parameters()

        # this variable is used to record the latest state of the current/last finished job
        self._response = gopher_delivery_srvs.DeliveryResultResponse()
        self._response.result = gopher_delivery_msgs.DeliveryErrorCodes.RESULT_NONE
        self._response.result_string = gopher_delivery_msgs.DeliveryErrorCodes.RESULT_NONE_STRING
        self._response_lock = threading.Lock()

        self.current_eta = gopher_delivery_msgs.DeliveryETA()  # connect to the planner to get ETA's

    def init(self):
        """
        Reusable initialisation for when we clear the delivery tree.
        """
        self.root = None
        self.subtrees = None
        self.init_blackboard_variables(traversed_locations=[], remaining_locations=[])

    @property
    def response(self):
        # Since the response is a thread protected variable...
        with self._response_lock:
            self._response.header.stamp = rospy.Time.now()
            response = copy.copy(self._response)
        return response

    @property
    def remaining_locations(self):
        return self.blackboard.remaining_locations

    @property
    def traversed_locations(self):
        return self.blackboard.traversed_locations

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

        self.subscribers = rocon_python_comms.utils.Subscribers(
            [
                ('eta', self.gopher.topics.eta, gopher_delivery_msgs.DeliveryETA, self._eta_callback)
            ]
        )
        self.services = rocon_python_comms.utils.Services(
            [
                ('delivery_result', self.gopher.services.delivery_result, gopher_delivery_srvs.DeliveryResult, self.get_last_goal_result),
                ('delivery_goal', self.gopher.services.delivery_goal, gopher_delivery_srvs.DeliveryGoal, self.set_goal)
            ]
        )
        self.subscribers = rocon_python_comms.utils.Subscribers(
            [
                ('current_world', self.gopher.topics.world, std_msgs.String, self.current_world_callback),
                ('delivery_cancel', self.gopher.topics.delivery_cancel, std_msgs.Empty, self.cancel_goal),
            ]
        )

        self.dynamic_reconfigure_server = dynamic_reconfigure.server.Server(
            QuirkyDeliveriesConfig,
            self.dynamic_reconfigure_callback
        )
        if len(self.semantics.locations) > 1:
            (deliveries_root, unused_subtrees) = create_delivery_subtree(
                world=self.semantics.worlds.default,
                locations=self.semantics.locations.keys()[:2]
            )
            self.ros_connected = deliveries_root.setup(timeout)
            if not self.ros_connected:
                rospy.logerr("Quirky Deliveries: failed to setup with the underlying ros subsystem")
        else:
            rospy.logerr("Quirky Deliveries: not enough locations listed in the semantics to support deliveries.")
            self.ros_connected = False

    def init_blackboard_variables(self, traversed_locations=[], remaining_locations=[]):
        self.blackboard.traversed_locations = traversed_locations
        self.blackboard.remaining_locations = remaining_locations
        self.blackboard.cancel_requested = False

    def dynamic_reconfigure_callback(self, config, level):
        self.parameters.express = config.express
        self.parameters.parking = config.parking
        conversions = {
            QuirkyDeliveriesConfig.QuirkyDeliveries_teleop: elf.InitialisationType.TELEOP,
            QuirkyDeliveriesConfig.QuirkyDeliveries_ar: elf.InitialisationType.AR,
        }
        self.parameters.elf = conversions[config.elf]
        rospy.loginfo("QuirkyDeliveries: reconfigured\n%s" % self.parameters)
        return config

    #################################
    # Goal Management
    #################################

    def set_goal(self, request):
        """
        Callback for receipt of a new goal. This can be called while the tree is ticking,
        so make sure you don't read/do anything with the blackboard here.

        This makes alot of checks to see whether it is viable to insert a new goal on
        the *next* tick. It doesn't actually do the insertion, yet.

         * check for empty goal
         * check underlying ros system is ready
         * check if there is enough battery
         * check goal semantic locations are registered ones
         * check the robot is not already busy/assigned

        :param [str] locations: semantic locations (try to match these against the system served Map Locations list via the unique_name)
        :return: True or False whether it was rejected or not
        """
        locations = request.semantic_locations
        rospy.loginfo("QuirkyDeliveries: received a goal request for {0}".format(request.semantic_locations))
        set_goal_response = gopher_delivery_srvs.DeliveryGoalResponse()
        with self._response_lock:
            # don't need a lock: http://effbot.org/pyfaq/what-kinds-of-global-value-mutation-are-thread-safe.htm
            if locations is None or not locations:
                set_goal_response.result = gopher_delivery_msgs.DeliveryErrorCodes.REJECT_GOAL_EMPTY_GOAL_NOTHING_TO_DO
                set_goal_response.error_message = "goal empty, nothing to do"
            elif not self.ros_connected:
                set_goal_response.result = gopher_delivery_msgs.DeliveryErrorCodes.REJECT_GOAL_SYSTEM_NOT_READY
                set_goal_response.error_message = "underlying ros subsystem not properly connected"
            elif self.blackboard.battery_low_warning:  # TODO this probably needs to be smarter
                set_goal_response.result = gopher_delivery_msgs.DeliveryErrorCodes.REJECT_GOAL_NOT_ENOUGH_JUICE
                set_goal_response.error_message = "not enough juice"
            elif self._response.result >= 0:
                if self.root is None:
                    # make sure locations are valid before returning success
                    if all([location in self.semantics.locations for location in locations]):
                        self.incoming_goal = locations
                        set_goal_response.result = gopher_delivery_msgs.DeliveryErrorCodes.SUCCESS
                        set_goal_response.error_message = "goal accepted"
                        self._response.result = gopher_delivery_msgs.DeliveryErrorCodes.RESULT_PENDING
                        self._response.result_string = gopher_delivery_msgs.DeliveryErrorCodes.RESULT_PENDING_STRING
                        self._response.message = "pending"
                    else:
                        set_goal_response.result = gopher_delivery_msgs.DeliveryErrorCodes.REJECT_GOAL_INVALID_LOCATION
                        set_goal_response.error_message = "received invalid locations"
                else:
                    set_goal_response.result = gopher_delivery_msgs.DeliveryErrorCodes.REJECT_GOAL_ROBOT_IS_BUSY
                    set_goal_response.error_message = "sorry, busy looking for a robot girl"
            elif self._response.result == gopher_delivery_msgs.DeliveryErrorCodes.RESULT_WAITING:
                set_goal_response.result = gopher_delivery_msgs.DeliveryErrorCodes.REJECT_GOAL_ROBOT_IS_BUSY
                set_goal_response.error_message = "pre-empting at waiting locations is not currently supported"
            else:
                set_goal_response.result = gopher_delivery_msgs.DeliveryErrorCodes.REJECT_GOAL_ALREADY_ASSIGNED
                set_goal_response.error_message = "sorry, busy (already assigned a goal)"

        if set_goal_response.result == gopher_delivery_msgs.DeliveryErrorCodes.SUCCESS:
            rospy.loginfo("QuirkyDeliveries: accepted goal request [%s]" % set_goal_response.error_message)
        else:
            rospy.logwarn("QuirkyDeliveries: rejected goal request [%s]" % set_goal_response.error_message)
        return set_goal_response

    def get_last_goal_result(self, request):
        """
        Since this is a service, it can be continuously polled, even while the delivery is running
        or after it has finished. This means it can indicate the result is still pending, or has
        finished some time ago (and a new one hasn't arrived yet).

        :param gopher_delivery_msgs.srv.DeliveryResultRequest request: last goal result details
        """
        return self.response  # note that callign this automatically activates the lock/copy

    def cancel_goal(self):
        """
        This will set a blackboard variable that will trigger a delivery to cleanup itself. Usually
        this is triggered by a guard that implements a subtree recovery at a higher priority than the actual delivery.
        """
        self.blackboard.cancel_requested = True

    def current_world_callback(self, msg):
        """
        Catches what world we are currently on from marco_polo. This is the unique name that
        should be found in the semantics worlds list. This is used to help the topological planner
        initialise a new delivery route.

        :param std_msgs.String msg:
        """
        self.current_world = msg.data

    #################################
    # Tick Tock
    #################################

    def pre_tick_update(self):
        """
        Check if we have a new goal, then assemble the sub-behaviours required to
        complete that goal. Finally kick out the current root if the last delivery
        succeeded.

        :returns: whether the tree needs to be inserted/deleted from above
        :rtype: bool
        """
        result = False  # BORING_NOTHING
        if self.incoming_goal is not None and self.incoming_goal:
            # use the planner to initialise the behaviours that we are to follow
            # to get to the delivery location. If this is empty/None, then the
            # semantic locations provided were wrong.
            self.old_goal_id = self.root.id if self.root is not None else None
            self.init_blackboard_variables(traversed_locations=[] if not self.root else self.blackboard.traversed_locations,
                                           remaining_locations=self.incoming_goal
                                           )
            (self.root, self.subtrees) = create_delivery_subtree(
                self.current_world,
                self.incoming_goal,
                self.parameters
            )

            if self.root.setup(10):
                self.locations = self.incoming_goal
                result = True  # NEW_DELIVERY_TREE
            else:
                rospy.logerr("Quirky Deliveries: failed to set up new delivery tree.")
                if self.root is None:
                    rospy.logerr("Quirky Deliveries: shouldn't ever get here, but in case you do...'git clone Dan; git pull'")
                result = False  # NEW_DELIVERY_TREE_WITHERED_AND_DIED
                self._response.result = gopher_delivery_msgs.DeliveryErrorCodes.RESULT_FAILED
                self._response.result_string = "delivery couldn't create the required behaviours to execute"
                self.init()
            self.incoming_goal = None

        elif self.root is not None and (self.root.status != py_trees.Status.RUNNING):
            # last goal was achieved, failed, or interrupted and no new goal, so swap this current subtree out
            self.old_goal_id = self.root.id if self.root is not None else None
            self.init()
            result = True  # TREE_FELLED
        return result

    def post_tock_update(self):
        """
        Save the state and set a feedback message that is useful for above.
        """
        # set the state - this is necessary so we have a variable outside the 'tick_tock' to handle
        # incoming calls on set_goal (comes from a ros service)
        if self.root is not None and self._response.result < 0:
            with self._response_lock:
                # just finished
                if self.root.status == py_trees.Status.SUCCESS:
                    self._response.result = gopher_delivery_msgs.DeliveryErrorCodes.SUCCESS
                    self._response.result_string = gopher_delivery_msgs.DeliveryErrorCodes.SUCCESS_STRING
                    self._response.message = "groot"
                # just aborted (interrupted)
                elif self.root.status == py_trees.Status.INVALID:
                    self._response.result = gopher_delivery_msgs.DeliveryErrorCodes.RESULT_ABORTED
                    self._response.result_string = gopher_delivery_msgs.DeliveryErrorCodes.RESULT_ABORTED_STRING
                    self._response.message = "aborted, probably by Marcus"
                elif self.subtrees.is_cancelled.status == py_trees.Status.SUCCESS:
                    self._response.result = gopher_delivery_msgs.DeliveryErrorCodes.RESULT_CANCELLED
                    self._response.result_string = gopher_delivery_msgs.DeliveryErrorCodes.RESULT_CANCELLED_STRING
                    self._response.message = "cancelled by the user"  # todo distinguish between concert and button cancel
                elif self.subtrees.deliveries.status == py_trees.Status.FAILURE:
                    self._response.result = gopher_delivery_msgs.DeliveryErrorCodes.RESULT_FAILED
                    self._response.result_string = gopher_delivery_msgs.DeliveryErrorCodes.RESULT_FAILED_STRING
                    self._response.message = "delivery failed, robot probably needs tlc"  # todo more detail
                elif self.subtrees.unparking.status == py_trees.common.Status.RUNNING:
                    self._response.result = gopher_delivery_msgs.DeliveryErrorCodes.RESULT_UNPARKING
                    self._response.result_string = gopher_delivery_msgs.DeliveryErrorCodes.RESULT_UNPARKING_STRING
                    self._response.message = gopher_delivery_msgs.DeliveryErrorCodes.RESULT_UNPARKING_STRING
                elif self.subtrees.en_route.status == py_trees.common.Status.RUNNING:
                    tip = self.root.tip()
                    if isinstance(tip, Waiting):
                        self._response.result = gopher_delivery_msgs.DeliveryErrorCodes.RESULT_WAITING
                        self._response.result_string = gopher_delivery_msgs.DeliveryErrorCodes.RESULT_WAITING_STRING
                        self._response.message = tip.feedback_message
                    else:
                        self._response.result = gopher_delivery_msgs.DeliveryErrorCodes.RESULT_TRAVELLING
                        self._response.result_string = gopher_delivery_msgs.DeliveryErrorCodes.RESULT_TRAVELLING_STRING
                        if self.blackboard.traversed_locations:
                            self._response.message = "moving from '{0}' to '{1}'".format(self.traversed_locations[-1], self.remaining_locations[0])
                        else:
                            self._response.message = "moving to '{0}'".format(self.remaining_locations[0])

                self._response.traversed_locations = self.traversed_locations
                self._response.remaining_locations = self.remaining_locations
                self._response.eta = self.current_eta

    #################################
    # Introspection
    #################################

    def has_subtree(self):
        return self.root is not None

    def is_running(self):
        if self.root is not None:
            return self.root.status == py_trees.Status.RUNNING
        return False

    #################################
    # Ros Callbacks
    #################################

    def _eta_callback(self, msg):
        self.current_eta = msg

#!/usr/bin/env python
#
# License: Yujin
#   https://raw.github.com/yujinrobot/gopher_crazy_hospital/license/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
Node that acts as caretaker of the behaviour tree for gopher deliveries.
"""
##############################################################################
# Imports
##############################################################################

import gopher_behaviours
import gopher_configuration
import gopher_delivery_msgs.srv as gopher_delivery_srvs
import gopher_delivery_msgs.msg as gopher_delivery_msgs
import move_base_msgs.msg as move_base_msgs
import std_msgs.msg as std_msgs
import py_trees
import actionlib
from py_trees.common import Status
import rocon_console.console as console
import rospy
import threading

##############################################################################
# Support
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
        self.express = rospy.get_param('~express', False)
        self.parking = rospy.get_param('~parking', False)

    def __str__(self):
        s = console.bold + "\nParameters:\n" + console.reset
        for key in sorted(self.__dict__):
            s += console.cyan + "    %s: " % key + console.yellow + "%s\n" % (self.__dict__[key] if self.__dict__[key] is not None else '-')
        s += console.reset
        return s

##############################################################################
# Core Controller
##############################################################################


class GopherHiveMind(object):
    def __init__(self):
        self.battery_subtree = gopher_behaviours.recovery.create_battery_recovery_tree(name="Eating Disorder")
        self.gopher = gopher_configuration.Configuration()
        self.current_world_subscriber = rospy.Subscriber(self.gopher.topics.world, std_msgs.String, self.current_world_callback)
        self.current_world = None
        self.quirky_deliveries = gopher_behaviours.delivery.GopherDeliveries(name="Quirky Deliveries")
        self._init_tree()
        self.logger = py_trees.logging.get_logger("HiveMind")
        self.success = None  # non goal
        self.cancelled = False
        self.cancelling = False
        self.cancelled_message = None
        self.current_eta = gopher_delivery_msgs.DeliveryETA()  # empty ETA
        self.monitor_ok = True

    def _init_tree(self):
        self.root = py_trees.Selector(name="HiveMind")

        #################################
        # Event Handler
        #################################
        self.event_handler = gopher_behaviours.interactions.create_button_event_handler()

        #################################
        # Global Abort
        #################################
        self.global_abort = py_trees.composites.Sequence(name="Global Abort")
        is_global_abort_activated = py_trees.CheckBlackboardVariable(
            name='Is Aborted?',
            variable_name='event_abort_button',
            expected_value=True
        )
        global_abort_repark = gopher_behaviours.park.create_repark_subtree()

        #################################
        # Idle
        #################################
        self.idle = py_trees.behaviours.Success("Idle")

        #################################
        # Blackboxes
        #################################
        self.global_abort.blackbox_level = py_trees.common.BlackBoxLevel.COMPONENT

        #################################
        # Graph
        #################################
        self.root.add_child(self.event_handler)
        self.root.add_child(self.global_abort)
        self.global_abort.add_child(is_global_abort_activated)
        self.global_abort.add_child(global_abort_repark)
        self.root.add_child(self.idle)
        self.tree = py_trees.ROSBehaviourTree(self.root)

    def render_dot_tree(self, visibility_level):
        """
        Fill in the tree with a 'default' delivery so we can render an exampler tree running
        with a delivery.
        """
        (deliveries_root, unused_delivery_subtree) = gopher_behaviours.delivery.create_delivery_subtree(
            world="earth",
            locations=["beer_fridge", "ashokas_hell"],
            express=False
        )
        self.tree.insert_subtree(deliveries_root, self.tree.root.id, 2)
        py_trees.display.render_dot_tree(self.tree.root, visibility_level)
        self.tree.prune_subtree(deliveries_root.id)

    ##################################
    # Ros Components
    ##################################

    def setup(self, timeout):
        """
        Delayed ros setup.
        """
        rospy.on_shutdown(self.shutdown)
        self.parameters = Parameters()
        self.quirky_deliveries.express = self.parameters.express
        self.quirky_deliveries.include_parking_behaviours = self.parameters.parking
        self.quirky_deliveries.setup(timeout)

        self._delivery_goal_service = rospy.Service('delivery/goal', gopher_delivery_srvs.DeliveryGoal, self._goal_service_callback)
        self._delivery_feedback_publisher = rospy.Publisher('delivery/feedback', gopher_delivery_msgs.DeliveryFeedback, queue_size=1, latch=True)
        self._status_pub = rospy.Publisher('~status', gopher_delivery_msgs.DeliveryManagerStatus, queue_size=1, latch=True)
        self._delivery_goal_cancel_sub = rospy.Subscriber('delivery/cancel', std_msgs.Empty, self._goal_cancel_callback)
        self._eta_sub = rospy.Subscriber('/navi/eta', gopher_delivery_msgs.DeliveryETA, self._eta_callback)
        self._delivery_result_service = rospy.Service('delivery/result', gopher_delivery_srvs.DeliveryResult, self._result_service_callback)
        self._status_pub.publish(gopher_delivery_msgs.DeliveryManagerStatus(status=gopher_delivery_msgs.DeliveryManagerStatus.IDLING))

        self.tree.setup(timeout)

        self.monitor_lock = threading.Lock()
        self.monitor_thread = threading.Thread(target=self.monitor, args=())
        self.monitor_thread.daemon = True
        self.monitor_thread.start()

    def monitor(self):
        '''
        Loop around looking for move base and break out once its found it.
        We assume that once found, it will always be there.

        .. todo::

           This should be much more comprehensive than just looking for a move base,
           Probably we should instantiate a hypothetical tree, call setup on that,
           and then drop it.
        '''
        action_client = actionlib.SimpleActionClient(self.gopher.actions.move_base, move_base_msgs.MoveBaseAction)
        while not rospy.is_shutdown():
            if not action_client.wait_for_server(rospy.Duration(30)):
                rospy.logwarn("Gopher Deliveries : could not find move base, retrying.")
                with self.monitor_lock:
                    self.monitor_ok = False
            else:
                with self.monitor_lock:
                    self.monitor_ok = True
                    break

    def _eta_callback(self, msg):
        self.current_eta = msg

    def current_world_callback(self, msg):
        """
        Catches what world we are currently on from marco_polo. This is the unique name that
        should be found in the semantics worlds list.
        """
        self.current_world = msg.data

    ##############################################################################
    # Tick Tock
    ##############################################################################

    def tick_tock(self):
        self.tree.visitors.append(py_trees.trees.DebugVisitor())
        self.tree.tick_tock(sleep_ms=500, number_of_iterations=py_trees.CONTINUOUS_TICK_TOCK, pre_tick_handler=self.pre_tick_handler, post_tick_handler=self.post_tick_handler)

    def pre_tick_handler(self, behaviour_tree):
        self.logger.debug("")
        self.logger.debug("{:-^30}".format(" Run %s " % behaviour_tree.count))
        self.logger.debug("")
        result = self.quirky_deliveries.pre_tick_update(self.current_world)
        if result == gopher_behaviours.delivery.PreTickResult.TREE_FELLED or result == gopher_behaviours.delivery.PreTickResult.NEW_DELIVERY_TREE:
            if self.quirky_deliveries.old_goal_id is not None:
                self.tree.prune_subtree(self.quirky_deliveries.old_goal_id)
            if self.quirky_deliveries.root:
                self.tree.insert_subtree(self.quirky_deliveries.root, self.root.id, 2)
            print("")
            print("************************************************************************************")
            print("                   Gopher Hivemind (Behaviour Tree Update)")
            print("************************************************************************************")
            py_trees.display.print_ascii_tree(self.tree.root)
            print("************************************************************************************")
            print("")
        elif result == gopher_behaviours.delivery.PreTickResult.NEW_DELIVERY_TREE_WITHERED_AND_DIED:
            self.cancelled = True
            self.cancelled_message = "Failed to grow the delivery subtree."

    def post_tick_handler(self, behaviour_tree):
        """
        Post results on the delivery feedback/results publishers.

        Could/should possibly move this into the delivery class itself.
        """
        self.quirky_deliveries.post_tock_update()
        # make sure to publish one last message to the feedback publisher once the task has succeded
        if self.quirky_deliveries.is_executing() or (self.quirky_deliveries.succeeded_on_last_tick() and not self.success):
            msg = gopher_delivery_msgs.DeliveryFeedback()
            msg.header.stamp = rospy.Time.now()
            msg.state = self.quirky_deliveries.state
            msg.traversed_locations = self.quirky_deliveries.blackboard.traversed_locations
            msg.remaining_locations = self.quirky_deliveries.blackboard.remaining_locations
            msg.status_message = self.quirky_deliveries.feedback_message
            msg.eta = self.current_eta
            self._delivery_feedback_publisher.publish(msg)
        if self.quirky_deliveries.completed_on_last_tick():
            if self.quirky_deliveries.succeeded_on_last_tick():
                self.success = True
            elif False:  # is the delivery cancelling?
                self.cancelling = False
                self.cancelled = True
                self.cancelled_message = "Delivery was cancelled (via teleop button or balcony scheduler)."
            else:
                # shouldn't get here
                pass
            self._status_pub.publish(gopher_delivery_msgs.DeliveryManagerStatus(status=gopher_delivery_msgs.DeliveryManagerStatus.IDLING))

        # If the battery subtree is running, we want to update the status, and
        # go back to idling once it succeeds. If it fails, then we are in limbo
        # and something is really broken anyway.
        if self.battery_subtree.status == py_trees.Status.RUNNING:
            self._status_pub.publish(gopher_delivery_msgs.DeliveryManagerStatus(status=gopher_delivery_msgs.DeliveryManagerStatus.BATTERY_LOW))
        elif self.battery_subtree.status == py_trees.Status.SUCCESS:
            self._status_pub.publish(gopher_delivery_msgs.DeliveryManagerStatus(status=gopher_delivery_msgs.DeliveryManagerStatus.IDLING))

    ##############################################################################
    # Ros Methods
    ##############################################################################

    def _goal_service_callback(self, request):
        '''
        Accept (or not) a delivery goal. Right now we refuse any goals assigned to us after the first.

        :param gopher_delivery_msgs.srv.DeliveryGoalRequest request: goal information

        .. todo::

           * check goal semantic locations are registered ones
           * check for homebase at front or back before pre-post fixing
           * check there is at least one location
           * check battery level via rocon_python_comms.ServiceProxy for a latched publisher
           * switch battery charging upon acceptance of a goal
           * check the first preempted goal location isn't the same as the current location and handle it
        '''
        rospy.loginfo("Delivery : received a goal request for {0}".format(request.semantic_locations))

        with self.monitor_lock:
            monitor_ok = self.monitor_ok

        # if the initialised variable is false, some monitored component is
        # unavailable - don't run a delivery
        if not monitor_ok:
            result = gopher_delivery_msgs.DeliveryErrorCodes.UNKNOWN
            message = "refused goal request: [Some components required for delivery aren't available yet.]"
            rospy.logwarn("Delivery : [%s]" % message)
        elif self.battery_subtree.status != Status.FAILURE:
            # don't accept anything if the battery subtree is doing something
            result = gopher_delivery_msgs.DeliveryErrorCodes.NOT_ENOUGH_JUICE
            message = "refused goal request : [Not enough battery power to accept a goal right now.]"
            rospy.logwarn("Delivery : [%s]" % message)
        else:
            (result, message) = self.quirky_deliveries.set_goal(request.semantic_locations, request.always_assume_initialised)
            if result == gopher_delivery_msgs.DeliveryErrorCodes.SUCCESS:
                message = "received goal request [%s]" % message
                rospy.loginfo("Delivery : [%s]" % message)
                self._status_pub.publish(gopher_delivery_msgs.DeliveryManagerStatus(status=gopher_delivery_msgs.DeliveryManagerStatus.DELIVERING))
                # reset flags upon confirming a new goal
                self.success = False
                self.cancelled = False
                self.cancelled_message = None
            else:
                message = "refused goal request [%s]" % message
                rospy.logwarn("Delivery : [%s]" % message)
        return gopher_delivery_srvs.DeliveryGoalResponse(result, message)

    def _goal_cancel_callback(self, cancel):
        # if the delivery root has been initialised
        if self.quirky_deliveries.root:
            # this doesn't do anything immediately, just flags a behaviour
            # node on so that the priority subtree changes
            self.quirky_deliveries.cancel_goal()
            self.cancelling = True

        msg = gopher_delivery_msgs.DeliveryFeedback()
        msg.header.stamp = rospy.Time.now()
        msg.state = gopher_delivery_msgs.DeliveryFeedback.CANCELLED
        msg.traversed_locations = self.quirky_deliveries.blackboard.traversed_locations
        msg.remaining_locations = self.quirky_deliveries.blackboard.remaining_locations
        msg.status_message = "Cancellation message was received."
        self._delivery_feedback_publisher.publish(msg)

    def _result_service_callback(self, request):
        '''
        Returns the result if success
        '''

        msg = gopher_delivery_srvs.DeliveryResultResponse()
        msg.header.stamp = rospy.Time.now()
        msg.traversed_locations = self.quirky_deliveries.blackboard.traversed_locations
        msg.remaining_locations = self.quirky_deliveries.blackboard.remaining_locations

        if self.success:
            msg.result = gopher_delivery_msgs.DeliveryErrorCodes.SUCCESS
            msg.error_message = "Success !"
        elif self.cancelled:
            msg.result = gopher_delivery_msgs.DeliveryErrorCodes.CANCELLED
            msg.error_message = self.cancelled_message
        elif self.cancelling:
            msg.result = gopher_delivery_msgs.DeliveryErrorCodes.UNKNOWN
            msg.error_message = "Cancelling..."
        else:
            msg.result = gopher_delivery_msgs.DeliveryErrorCodes.UNKNOWN
            msg.error_message = "Not finished yet..."
        return msg

    def shutdown(self):
        self.tree.destroy()  # destroy the tree on shutdown to stop the behaviour
        self.tree.interrupt()

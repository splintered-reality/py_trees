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
import std_msgs.msg as std_msgs
import py_trees
import rocon_console.console as console
import rocon_python_comms
import rospy

from . import battery

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
        self.logger = py_trees.logging.get_logger("HiveMind")
        self.gopher = gopher_configuration.Configuration()
        self.current_world = None
        self.quirky_deliveries = gopher_behaviours.delivery.GopherDeliveries(name="Quirky Deliveries")
        self.current_eta = gopher_delivery_msgs.DeliveryETA()  # empty ETA
        self.response = gopher_delivery_srvs.DeliveryResultResponse()
        self.response.result = gopher_delivery_msgs.DeliveryErrorCodes.RESULT_UNKNOWN
        self.response.error_message = ""

    def _init_tree(self):
        self.gopher = gopher_configuration.Configuration(fallback_to_defaults=True)
        self.root = py_trees.composites.Parallel(
            name="HiveMind",
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL
        )
        self.priorities = py_trees.Selector(name="Priorities")

        #################################
        # Event Handling & Blackboard
        #################################
        event_handler = gopher_behaviours.interactions.create_button_event_handler()
        battery_to_blackboard = battery.ToBlackboard(name="Battery2BB", topic_name=self.gopher.topics.battery)

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
        self.root.add_child(event_handler)
        self.root.add_child(battery_to_blackboard)
        self.root.add_child(self.priorities)
        self.priorities.add_child(self.global_abort)
        self.global_abort.add_child(is_global_abort_activated)
        self.global_abort.add_child(global_abort_repark)
        self.priorities.add_child(self.idle)
        self.tree = py_trees.ROSBehaviourTree(self.root)

    def render_dot_tree(self, visibility_level):
        """
        Fill in the tree with a 'default' delivery so we can render an exampler tree running
        with a delivery.
        """
        (deliveries_root, unused_deliveries, unused_en_route, unused_is_cancelled) = gopher_behaviours.delivery.create_delivery_subtree(
            world="earth",
            locations=["beer_fridge", "ashokas_hell"],
            express=False
        )
        self.tree.insert_subtree(deliveries_root, self.priorities.id, 1)
        py_trees.display.render_dot_tree(self.tree.root, visibility_level)
        self.tree.prune_subtree(deliveries_root.id)

    ##################################
    # Ros Components
    ##################################

    def setup(self, timeout):
        """
        Delayed ros setup.
        """
        self._init_tree()

        rospy.on_shutdown(self.shutdown)
        self.parameters = Parameters()
        self.quirky_deliveries.express = self.parameters.express
        self.quirky_deliveries.include_parking_behaviours = self.parameters.parking
        self.quirky_deliveries.setup(timeout)
        self.behaviour_status = gopher_delivery_msgs.BehaviourReport.BUSY

        latched = True
        queue_size_five = 5
        self.publishers = rocon_python_comms.utils.Publishers(
            [
                ('feedback', self.gopher.topics.delivery_feedback, gopher_delivery_msgs.DeliveryFeedback, latched, queue_size_five),
                ('report', self.gopher.topics.behaviour_report, gopher_delivery_msgs.BehaviourReport, latched, queue_size_five),
            ]
        )
        self.services = rocon_python_comms.utils.Services(
            [
                ('delivery_result', self.gopher.services.delivery_result, gopher_delivery_srvs.DeliveryResult, self._result_service_callback),
                ('delivery_goal', self.gopher.services.delivery_goal, gopher_delivery_srvs.DeliveryGoal, self._goal_service_callback)
            ]
        )
        self.subscribers = rocon_python_comms.utils.Subscribers(
            [
                ('current_world', self.gopher.topics.world, std_msgs.String, self.current_world_callback),
                ('delivery_cancel', self.gopher.topics.delivery_cancel, std_msgs.Empty, self._goal_cancel_callback),
                ('eta', self.gopher.topics.eta, gopher_delivery_msgs.DeliveryETA, self._eta_callback),
            ]
        )

        self.publishers.report.publish(gopher_delivery_msgs.BehaviourReport(status=self.behaviour_status))
        self.tree.setup(timeout)

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
                self.tree.insert_subtree(self.quirky_deliveries.root, self.priorities.id, 1)
            print("")
            print("************************************************************************************")
            print("                   Gopher Hivemind (Behaviour Tree Update)")
            print("************************************************************************************")
            py_trees.display.print_ascii_tree(self.tree.root)
            print("************************************************************************************")
            print("")
        elif result == gopher_behaviours.delivery.PreTickResult.NEW_DELIVERY_TREE_WITHERED_AND_DIED:
            self.response.result = gopher_delivery_msgs.DeliveryErrorCodes.RESULT_FAILED
            self.response.error_message = "delivery couldn't create the required behaviours to execute"

    def post_tick_handler(self, behaviour_tree):
        """
        Update delivery results, manager status and post on publishers.
        """
        self.quirky_deliveries.post_tock_update()
        # Check for delivery result update
        if self.response.result == gopher_delivery_msgs.DeliveryErrorCodes.RESULT_PENDING:
            if self.quirky_deliveries.is_interrupted():
                self.response.result = gopher_delivery_msgs.DeliveryErrorCodes.RESULT_ABORTED
                self.response.error_message = "delivery aborted"
            elif self.quirky_deliveries.is_running_but_cancelled():
                self.response.result = gopher_delivery_msgs.DeliveryErrorCodes.RESULT_CANCELLED
                self.response.error_message = "delivery cancelled"
            elif self.quirky_deliveries.is_running_but_failed():
                self.response.result = gopher_delivery_msgs.DeliveryErrorCodes.RESULT_FAILED
                self.response.error_message = "delivery failed"
            elif self.quirky_deliveries.is_finished():
                self.response.result = gopher_delivery_msgs.DeliveryErrorCodes.SUCCESS
                self.response.error_message = "success"
        # Feedback
        if self.quirky_deliveries.is_running():
            msg = gopher_delivery_msgs.DeliveryFeedback()
            msg.header.stamp = rospy.Time.now()
            msg.state = str(self.quirky_deliveries.state)
            msg.traversed_locations = self.quirky_deliveries.blackboard.traversed_locations
            msg.remaining_locations = self.quirky_deliveries.blackboard.remaining_locations
            msg.status_message = self.quirky_deliveries.feedback_message
            msg.eta = self.current_eta
            self.publishers.feedback.publish(msg)

        # Manager Status
        old_status = self.behaviour_status
        if self.idle.status == py_trees.common.Status.SUCCESS:
            self.behaviour_status = gopher_delivery_msgs.BehaviourReport.READY
        elif self.response.result == gopher_delivery_msgs.DeliveryErrorCodes.RESULT_PENDING:
            self.behaviour_status = gopher_delivery_msgs.BehaviourReport.DELIVERING
        else:  # it's aborting, failed setup, or it's busy cancelling, parking or something
            self.behaviour_status = gopher_delivery_msgs.BehaviourReport.BUSY
        if old_status != self.behaviour_status:
            self.publishers.report.publish(gopher_delivery_msgs.BehaviourReport(status=self.behaviour_status))

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

        (result, message) = self.quirky_deliveries.set_goal(request.semantic_locations)
        if result == gopher_delivery_msgs.DeliveryErrorCodes.SUCCESS:
            message = "received goal request [%s]" % message
            rospy.loginfo("Delivery : [%s]" % message)
            self.response.result = gopher_delivery_msgs.DeliveryErrorCodes.RESULT_PENDING
            self.response.error_message = 'pending'
        else:
            message = "refused goal request [%s]" % message
            rospy.logwarn("Delivery : [%s]" % message)
        return gopher_delivery_srvs.DeliveryGoalResponse(result, message)

    def _goal_cancel_callback(self, cancel):
        """
        This doesn't do anything immediately, just flags a behaviour
        node on so that the priority subtree changes.
        """
        if self.quirky_deliveries.has_subtree():
            self.quirky_deliveries.cancel_goal()

    def _result_service_callback(self, request):
        '''
        Returns the result if success
        '''
        self.response.header.stamp = rospy.Time.now()
        self.response.traversed_locations = self.quirky_deliveries.blackboard.traversed_locations
        self.response.remaining_locations = self.quirky_deliveries.blackboard.remaining_locations
        return self.response

    def shutdown(self):
        self.tree.destroy()  # destroy the tree on shutdown to stop the behaviour
        self.tree.interrupt()

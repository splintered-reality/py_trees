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

import geometry_msgs.msg as geometry_msgs
import gopher_behaviours
import gopher_configuration
import gopher_delivery_msgs.srv as gopher_delivery_srvs
import gopher_delivery_msgs.msg as gopher_delivery_msgs
import nav_msgs.msg as nav_msgs
import py_trees
import rocon_console.console as console
import rocon_python_comms
import rospy

from . import battery

##############################################################################
# Core Controller
##############################################################################


class SplinteredReality(object):
    def __init__(self):
        self.logger = py_trees.logging.get_logger("HiveMind")
        self.quirky_deliveries = gopher_behaviours.delivery.GopherDeliveries(name="Quirky Deliveries")
        self.response = gopher_delivery_srvs.DeliveryResultResponse()
        self.response.result = gopher_delivery_msgs.DeliveryErrorCodes.RESULT_NONE
        self.response.result_string = gopher_delivery_msgs.DeliveryErrorCodes.RESULT_NONE_STRING

    def _init_tree(self):
        self.gopher = gopher_configuration.Configuration(fallback_to_defaults=True)
        self.root = py_trees.composites.Parallel(
            name="Splintered Reality",
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL
        )
        self.priorities = py_trees.Selector(name="Priorities")

        #################################
        # Event Handling & Blackboard
        #################################
        event_handler = gopher_behaviours.interactions.create_button_event_handler()
        topics_to_blackboard = py_trees.composites.Sequence(name="Topics2BB")
        battery_to_blackboard = battery.ToBlackboard(name="Battery2BB", topic_name=self.gopher.topics.battery)
        odom_to_blackboard = py_trees.subscribers.ToBlackboard(
            name="Odom2BB",
            topic_name=self.gopher.topics.odom,
            topic_type=nav_msgs.Odometry,
            blackboard_variables={"odom": None},
            clearing_policy=py_trees.common.ClearingPolicy.NEVER
        )
        pose_to_blackboard = py_trees.subscribers.ToBlackboard(
            name="Pose2BB",
            topic_name=self.gopher.topics.pose,
            topic_type=geometry_msgs.PoseWithCovarianceStamped,
            blackboard_variables={"pose": None},
            clearing_policy=py_trees.common.ClearingPolicy.NEVER
        )

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
        # Init
        #################################
        low_priority_homebase_initialisation = py_trees.composites.Sequence(name="Homebase Initialisation")
        is_init_activated = py_trees.CheckBlackboardVariable(
            name='Is Inited?',
            variable_name='event_init_button',
            expected_value=True
        )
        teleop_and_teleport = gopher_behaviours.navigation.create_teleop_homebase_teleport_subtree()
        hang_around = py_trees.timers.Timer(name="Hang Around", duration=2.0)

        #################################
        # Idle
        #################################
        self.idle = py_trees.behaviours.Success("Idle")

        #################################
        # Blackboxes
        #################################
        topics_to_blackboard.blackbox_level = py_trees.common.BlackBoxLevel.DETAIL
        self.global_abort.blackbox_level = py_trees.common.BlackBoxLevel.COMPONENT
        self.low_priority_homebase_initialisation = py_trees.common.BlackBoxLevel.COMPONENT

        #################################
        # Graph
        #################################
        self.root.add_child(event_handler)
        self.root.add_child(topics_to_blackboard)
        topics_to_blackboard.add_child(battery_to_blackboard)
        topics_to_blackboard.add_child(odom_to_blackboard)
        topics_to_blackboard.add_child(pose_to_blackboard)
        self.root.add_child(self.priorities)
        self.priorities.add_child(self.global_abort)
        self.global_abort.add_child(is_global_abort_activated)
        self.global_abort.add_child(global_abort_repark)
        self.priorities.add_child(low_priority_homebase_initialisation)
        low_priority_homebase_initialisation.add_child(is_init_activated)
        low_priority_homebase_initialisation.add_child(teleop_and_teleport)
        low_priority_homebase_initialisation.add_child(hang_around)
        self.priorities.add_child(self.idle)
        self.tree = py_trees.ROSBehaviourTree(self.root)

    def render_dot_tree(self, visibility_level):
        """
        Fill in the tree with a 'default' delivery so we can render an exampler tree running
        with a delivery.
        """
        (deliveries_root, unused_subtrees) = gopher_behaviours.delivery.create_delivery_subtree(
            world="earth",
            locations=["beer_fridge", "ashokas_hell"]
        )
        self._init_tree()
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
        self.quirky_deliveries.setup(timeout)
        self.behaviour_status = gopher_delivery_msgs.BehaviourReport.BUSY

        latched = True
        queue_size_five = 5
        self.publishers = rocon_python_comms.utils.Publishers(
            [
                ('report', self.gopher.topics.behaviour_report, gopher_delivery_msgs.BehaviourReport, latched, queue_size_five),
            ]
        )

        self.publishers.report.publish(gopher_delivery_msgs.BehaviourReport(status=self.behaviour_status))
        self.tree.setup(timeout)

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
        if self.quirky_deliveries.pre_tick_update():
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

    def post_tick_handler(self, behaviour_tree):
        """
        Update delivery results, manager status and post on publishers.
        """
        # Deliveries result update
        self.quirky_deliveries.post_tock_update()

        # Manager Status
        old_status = self.behaviour_status
        if self.idle.status == py_trees.common.Status.SUCCESS:
            self.behaviour_status = gopher_delivery_msgs.BehaviourReport.READY
        elif self.response.result < 0:
            self.behaviour_status = gopher_delivery_msgs.BehaviourReport.DELIVERING
        else:  # it's aborting, failed setup, or it's busy cancelling, parking or something
            self.behaviour_status = gopher_delivery_msgs.BehaviourReport.BUSY
        if old_status != self.behaviour_status:
            self.publishers.report.publish(gopher_delivery_msgs.BehaviourReport(status=self.behaviour_status))

    ##############################################################################
    # Ros Methods
    ##############################################################################

    def _goal_cancel_callback(self, cancel):
        """
        This doesn't do anything immediately, just flags a behaviour
        node on so that the priority subtree changes.
        """
        if self.quirky_deliveries.has_subtree():
            self.quirky_deliveries.cancel_goal()

    def shutdown(self):
        self.tree.destroy()  # destroy the tree on shutdown to stop the behaviour
        self.tree.interrupt()

#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/yujinrobot/gopher_crazy_hospital/license/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
.. module:: compulsion
   :platform: Unix
   :synopsis: Mind control the little suckers.

Coerce the gophers into doing what you want them to do. Just a little bit
of planning and behavioural compulsion.

----

"""

##############################################################################
# Imports
##############################################################################

import gopher_std_msgs.msg as gopher_std_msgs
import gopher_std_msgs.srv as gopher_std_srvs
import logging
import py_trees
import rospy

from . import battery
from . import delivery

##############################################################################
# Classes
##############################################################################


class Visitor:
    def __init__(self):
        self.logger = logging.getLogger("gopher_behaviours.Visitor")

    def run(self, behaviour):
        if behaviour.feedback_message:
            self.logger.debug("  %s [visited][%s][%s]" % (behaviour.name, behaviour.status, behaviour.feedback_message))
        else:
            self.logger.debug("  %s [visited][%s]" % (behaviour.name, behaviour.status))


class PreTickVisitor:
    def __init__(self):
        self.logger = logging.getLogger("gopher_behaviours.PreTick")

    def run(self, behaviour_tree):
        self.logger.debug("\n--------- Run %s ---------\n" % behaviour_tree.count)


class GopherCompulsion(object):
    def __init__(self):
        self.battery_subtree = battery.create_battery_tree(name="Eating Disorder")
        self.quirky_deliveries = delivery.GopherDeliveries(name="Quirky Deliveries", locations=["beer_fridge", "pizza_shop", "sofa_in_front_of_tv", "anywhere_not_near_the_wife"])
        self.idle = py_trees.behaviours.Success("Idle")
        self.root = py_trees.Selector(name="Gopher Compulsion", children=[self.battery_subtree, self.idle])
        self.tree = py_trees.BehaviourTree(self.root)

        ##################################
        # Ros Components
        ##################################
        rospy.on_shutdown(self.shutdown)
        self._delivery_goal_service = rospy.Service('delivery/goal', gopher_std_srvs.DeliveryGoal, self._goal_service_callback)
        self._delivery_feedback_publisher = rospy.Publisher('delivery/feedback', gopher_std_msgs.DeliveryFeedback, queue_size=1, latch=True)
        self._delivery_result_publisher = rospy.Publisher('delivery/result', gopher_std_msgs.DeliveryResult, queue_size=1, latch=True)

    def tick_tock(self):
        self.tree.visitors.append(Visitor())
        self.tree.tick_tock(sleep_ms=500, number_of_iterations=py_trees.CONTINUOUS_TICK_TOCK, pre_tick_visitor=self.pre_tick_visitor, post_tock_visitor=self.post_tock_visitor)

    def pre_tick_visitor(self, behaviour_tree):
        self.quirky_deliveries.pre_tick_update()
        if self.quirky_deliveries.has_a_new_goal:
            if self.quirky_deliveries.old_goal_id is not None:
                self.tree.prune_subtree(self.quirky_deliveries.old_goal_id)
            if self.quirky_deliveries.root:
                self.tree.insert_subtree(self.quirky_deliveries.root, self.root.id, 1)
            self.quirky_deliveries.has_a_new_goal = False
            print("")
            print("************************************************************************************")
            print("                   Gopher Compelled (Behaviour Tree Update)")
            print("************************************************************************************")
            py_trees.display.print_ascii_tree(self.tree.root)
            print("************************************************************************************")
            print("")

    def _goal_service_callback(self, request):
        '''
        Accept (or not) a delivery goal. Right now we refuse any goals assigned to us after the first.

        :param gopher_std_msgs.srv.DeliveryGoalRequest request: goal information

        .. todo::

           * check goal semantic locations are registered ones
           * check for homebase at front or back before pre-post fixing
           * check there is at least one location
           * check battery level via rocon_python_comms.ServiceProxy for a latched publisher
           * switch battery charging upon acceptance of a goal
           * check the first preempted goal location isn't the same as the current location and handle it
        '''
        rospy.loginfo("Delivery : received a goal request: {0}".format(request))
        (result, message) = self.quirky_deliveries.set_goal(request.semantic_locations)
        if result == gopher_std_msgs.DeliveryErrorCodes.SUCCESS:
            message = "received goal request [%s]" % message
            rospy.loginfo("Delivery : " % message)
        else:
            message = "refused goal request [%s]" % message
            rospy.logwarn("Delivery : " % message)
        return gopher_std_srvs.DeliveryGoalResponse(result, message)

    def shutdown(self):
        self.tree.interrupt_tick_tocking = True

    def post_tock_visitor(self, behaviour_tree):
        self.quirky_deliveries.post_tock_update()

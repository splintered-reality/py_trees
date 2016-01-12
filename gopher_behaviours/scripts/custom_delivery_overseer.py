#!/usr/bin/env python
#
# License: Yujin
#   https://raw.github.com/yujinrobot/gopher_crazy_hospital/license/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
Node that responds to triggers for initiation and execution of custom
deliveries.
"""

##############################################################################
# Imports
##############################################################################

import gopher_behaviours
import gopher_configuration
import gopher_delivery_msgs.msg as gopher_delivery_msgs
import gopher_navi_msgs.msg as gopher_navi_msgs
import rocon_python_comms
import rospy
import std_msgs.msg as std_msgs

##############################################################################
# Classes
##############################################################################


class CustomDeliveryOverseer():
    """
    **Subscribers**

     * **~delivery_manager_status** (*gopher_delivery_msgs.DeliveryManagerStatus*) - listens to the state (idle/delivering) of the deliver manager to know when it is busy/not.
     * **/gopher/buttons/go** (*std_msgs.Empty*) - will initiate and execute a delivery when a joystick/button event comes in on this topic.

    """
    def __init__(self):
        self.gopher = gopher_configuration.Configuration()
        self.delivering = False  # is the delivery manager busy?
        self.custom_delivering = False  # are we doing something ourselves?
        self.locations = rospy.get_param('~locations')  # assumed to be a list of strings
        rospy.loginfo("Custom Delivery : setup for [" + "->".join(self.locations) + "] deliveries.")
        self.express_delivery = gopher_behaviours.scripts.deliveries.ExpressDelivery(verbose_feedback=False)
        self.subscribers = rocon_python_comms.utils.Subscribers(
            [
                ('delivery_manager_status', '~delivery_manager_status', gopher_delivery_msgs.DeliveryManagerStatus, self.delivery_manager_status_cb),
                ('trigger', self.gopher.buttons.go, std_msgs.Empty, self.go_cb),
                ('init_homebase', self.gopher.buttons.stop, std_msgs.Empty, self.stop_cb)
            ]
        )
        not_latched = False
        self.publishers = rocon_python_comms.utils.Publishers(
            [
                ('teleport_homebase', self.gopher.actions.teleport + "/goal", gopher_navi_msgs.TeleportActionGoal, not_latched, 5),
            ]
        )

    def delivery_manager_status_cb(self, msg):
        self.delivering = (msg.status != gopher_delivery_msgs.DeliveryManagerStatus.IDLING)

    def go_cb(self, unused_msg):
        if not self.delivering:
            rospy.loginfo("Custom Delivery : custom delivery triggered via joystick and initiated.")
            self.express_delivery.send(self.locations, include_parking_behaviours=False)
            # Small chance of race condition here since it now competes with the official
            # source of deliveries from the concert. The user should be aware of this and
            # make certain he is using one...or the other.
            self.custom_delivering = True
        else:
            if not self.custom_delivering:
                rospy.logwarn("Custom Delivery : rejecting request as the delivery manager is already busy.")
            else:
                rospy.loginfo("Custom Delivery : accepting a request to do custom delivery.")

    def stop_cb(self, unused_msg):
        if not self.delivering:
            rospy.loginfo("Custom Delivery : init'ing the robot on the homebase.")
            action_goal = gopher_navi_msgs.TeleportActionGoal()
            action_goal.header.stamp = rospy.Time.now()
            goal = gopher_navi_msgs.TeleportGoal()
            goal.location = "homebase"
            goal.special_effects = True
            action_goal.goal = goal
            self.publishers.teleport_homebase.publish(action_goal)

    def spin(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.custom_delivering:
                self.express_delivery.spin()  # blocking until the delivery is finished
                self.custom_delivering = False
                rospy.loginfo("Custom Delivery : finished a delivery.")
            else:
                try:
                    rate.sleep()
                except rospy.exceptions.ROSInterruptException:  # ros shutting down
                    break

if __name__ == '__main__':
    rospy.init_node("custom_delivery_overseer", log_level=rospy.INFO)
    custom_delivery_overseer = CustomDeliveryOverseer()
    custom_delivery_overseer.spin()

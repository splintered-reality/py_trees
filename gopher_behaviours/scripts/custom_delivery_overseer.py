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

import dynamic_reconfigure.server
import gopher_behaviours
from gopher_behaviours.cfg import CustomDeliveriesConfig
import gopher_configuration
import gopher_delivery_msgs.msg as gopher_delivery_msgs
import gopher_navi_msgs.msg as gopher_navi_msgs
import gopher_semantics
import rocon_console.console as console
import rocon_python_comms
import rospy
import std_msgs.msg as std_msgs

##############################################################################
# Classes
##############################################################################


class CustomDeliveryOverseer(object):
    """
    **Subscribers**

     * **~delivery_manager_status** (*gopher_delivery_msgs.DeliveryManagerStatus*) - listens to the state (idle/delivering) of the deliver manager to know when it is busy/not.
     * **/gopher/buttons/go** (*std_msgs.Empty*) - will initiate and execute a delivery when a joystick/button event comes in on this topic.

    """
    def __init__(self):
        self.gopher = gopher_configuration.Configuration()
        self.semantics = gopher_semantics.Semantics(self.gopher.namespaces.semantics)
        self.delivering = False  # is the delivery manager busy?
        self.custom_delivering = False  # are we doing something ourselves?
        self.locations = None
        self.dynamic_reconfigure_server =\
            dynamic_reconfigure.server.Server(CustomDeliveriesConfig, self.dynamic_reconfigure_cb)
        self.express_delivery = gopher_behaviours.scripts.deliveries.CustomDelivery(verbose_feedback=False)
        self.subscribers = rocon_python_comms.utils.Subscribers(
            [
                ('delivery_manager_status', '~delivery_manager_status', gopher_delivery_msgs.DeliveryManagerStatus, self.delivery_manager_status_cb),
                ('trigger', self.gopher.topics.delivery_custom, std_msgs.Empty, self.delivery_custom_trigger_cb),
                # ('init_homebase', self.gopher.buttons.stop, std_msgs.Empty, self.stop_cb)
            ]
        )
#        not_latched = False
#         self.publishers = rocon_python_comms.utils.Publishers(
#             [
#                 ('teleport_homebase', self.gopher.actions.teleport + "/goal", gopher_navi_msgs.TeleportActionGoal, not_latched, 5),
#             ]
#         )

    def delivery_manager_status_cb(self, msg):
        self.delivering = (msg.status != gopher_delivery_msgs.DeliveryManagerStatus.IDLING)

    def delivery_custom_trigger_cb(self, unused_msg):
        if not self.delivering:
            rospy.loginfo("Custom Delivery : custom delivery triggered via joystick and initiated.")
            self.express_delivery.send(self.locations, include_parking_behaviours=False)
            # Small chance of race condition here since it now competes with the official
            # source of deliveries from the concert. The user should be aware of this and
            # make certain he is using one...or the other.
            self.custom_delivering = True
        else:
            rospy.loginfo("Custom Delivery : rejecting request as the delivery manager is already busy.")

#     def stop_cb(self, unused_msg):
#         if not self.delivering:
#             rospy.loginfo("Custom Delivery : init'ing the robot on the homebase.")
#             action_goal = gopher_navi_msgs.TeleportActionGoal()
#             action_goal.header.stamp = rospy.Time.now()
#             goal = gopher_navi_msgs.TeleportGoal()
#             goal.location = "homebase"
#             goal.special_effects = True
#             action_goal.goal = goal
#             self.publishers.teleport_homebase.publish(action_goal)

    def dynamic_reconfigure_cb(self, config, level):
        """
        Don't use the incoming config except as a read only
        tool. If we write variables, make sure to use the server's update_configuration, which will trigger
        this callback here.
        """
        temp_string = config.locations
        temp_string = temp_string.replace('[', ' ')
        temp_string = temp_string.replace(']', ' ')
        temp_string = temp_string.replace(',', ' ')
        new_locations = [l for l in temp_string.split(' ') if l]
        self.locations = []
        for location in new_locations:
            if location in self.semantics.locations:
                self.locations.append(location)
            else:
                rospy.logerr("Custom Delivery: tried to reconfigure with an invalid location, dropping [%s]" % location)
        rospy.loginfo("Custom Delivery: setup for [" + "->".join(self.locations) + "] deliveries.")
        return config

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

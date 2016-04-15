#!/usr/bin/env python
#
# License: Unspecified
#
##############################################################################
# Imports
##############################################################################

import datetime
import gopher_configuration
import gopher_delivery_msgs.msg as gopher_delivery_msgs
import gopher_delivery_msgs.srv as gopher_delivery_srvs
import rocon_console.console as console
import rospy
import sys

##############################################################################
# CustomDelivery
##############################################################################


class CustomDelivery(object):
    def __init__(self):
        self.gopher = gopher_configuration.configuration.Configuration(fallback_to_defaults=True)

    def send(self, goal_locations):
        delivery_goal_request = gopher_delivery_srvs.DeliveryGoalRequest()
        delivery_goal_request.semantic_locations = goal_locations
        rospy.loginfo("CustomDelivery: sending request %s" % delivery_goal_request.semantic_locations)
        delivery_goal_service = rospy.ServiceProxy(self.gopher.services.delivery_goal, gopher_delivery_srvs.DeliveryGoal)
        try:
            unused_delivery_goal_response = delivery_goal_service(delivery_goal_request)
            if not unused_delivery_goal_response.result == 0:
                print(console.red + "CustomDelivery: goal service call failed: %s" % unused_delivery_goal_response.message + console.reset)
                sys.exit()
        except rospy.ServiceException, e:
            print(console.red + "CustomDelivery: goal service call failed: %s" % e + console.reset)
            sys.exit()
        except rospy.exceptions.ROSInterruptException:
            sys.exit()  # ros shutting down
        rospy.wait_for_service(self.gopher.services.delivery_result)

    def spin(self):
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            try:
                fetch_result = rospy.ServiceProxy(self.gopher.services.delivery_result, gopher_delivery_srvs.DeliveryResult)
                response = fetch_result()
                if response.result >= 0:
                    CustomDelivery.result(response)
                    break
                rate.sleep()
            except rospy.ServiceException, e:
                print(console.red + "CustomDelivery: service call failed: %s" % e + console.reset)
                break
            except rospy.exceptions.ROSInterruptException:  # ros shutting down
                break

    @staticmethod
    def result(msg):
        print("")
        print(console.bold + "Delivery Result" + console.reset)
        date_string = datetime.datetime.fromtimestamp(int(msg.header.stamp.to_sec())).strftime('%Y-%m-%d %H:%M:%S')
        print(console.cyan + "  timestamp    : " + console.yellow + "%s" % date_string + console.reset)
        print(console.cyan + "  result       : " + console.yellow + "%s" % msg.result + console.reset)
        print(console.cyan + "  result_string: " + console.yellow + "%s" % msg.result_string + console.reset)
        print(console.cyan + "  message      : " + console.yellow + "%s" % msg.message + console.reset)
        print(console.cyan + "  traversed    : " + console.yellow + "%s" % msg.traversed_locations + console.reset)
        print(console.cyan + "  remaining    : " + console.yellow + "%s" % msg.remaining_locations + console.reset)
        print(console.cyan + "  eta          : " + console.yellow + "%s" % msg.eta + console.reset)
        print("")

##############################################################################
# Classes
##############################################################################

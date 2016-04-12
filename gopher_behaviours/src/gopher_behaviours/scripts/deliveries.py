#!/usr/bin/env python
#
# License: Unspecified
#
##############################################################################
# Imports
##############################################################################

import datetime
import gopher_delivery_msgs.msg as gopher_delivery_msgs
import gopher_delivery_msgs.srv as gopher_delivery_srvs
import rocon_console.console as console
import rospy
import sys

##############################################################################
# CustomDelivery
##############################################################################


class CustomDelivery(object):
    def __init__(self, verbose_feedback=True):
        if verbose_feedback:
            self.feedback_subscriber = rospy.Subscriber("/rocon/delivery/feedback", gopher_delivery_msgs.DeliveryFeedback, CustomDelivery.feedback)

    def send(self, goal_locations, include_parking_behaviours):
        delivery_goal_request = gopher_delivery_srvs.DeliveryGoalRequest()
        delivery_goal_request.semantic_locations = goal_locations
        delivery_goal_request.always_assume_initialised = not include_parking_behaviours
        delivery_goal_request.cycle_door = False
        rospy.loginfo("Express Deliveries : sending request %s" % delivery_goal_request.semantic_locations)
        delivery_goal_service = rospy.ServiceProxy("/rocon/delivery/goal", gopher_delivery_srvs.DeliveryGoal)
        try:
            unused_delivery_goal_response = delivery_goal_service(delivery_goal_request)
            if not unused_delivery_goal_response.result == 0:
                print(console.red + "Delivery goal service call failed: %s" % unused_delivery_goal_response.error_message + console.reset)
                sys.exit()
        except rospy.ServiceException, e:
            print(console.red + "Delivery goal service call failed: %s" % e + console.reset)
            sys.exit()
        except rospy.exceptions.ROSInterruptException:
            sys.exit()  # ros shutting down
        rospy.wait_for_service('/rocon/delivery/result')

    def spin(self):
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            try:
                fetch_result = rospy.ServiceProxy('/rocon/delivery/result', gopher_delivery_srvs.DeliveryResult)
                response = fetch_result()
                if response.result != gopher_delivery_msgs.DeliveryErrorCodes.RESULT_PENDING:
                    CustomDelivery.result(response)
                    break
                rate.sleep()
            except rospy.ServiceException, e:
                print(console.red + "Service call failed: %s" % e + console.reset)
                break
            except rospy.exceptions.ROSInterruptException:  # ros shutting down
                break

    @staticmethod
    def feedback(msg):
        print("")
        print(console.bold + "Delivery Feedback" + console.reset)
        date_string = datetime.datetime.fromtimestamp(int(msg.header.stamp.to_sec())).strftime('%Y-%m-%d %H:%M:%S')
        print(console.cyan + "  timestamp: " + console.yellow + "%s" % date_string + console.reset)
        print(console.cyan + "  traversed: " + console.yellow + "%s" % msg.traversed_locations + console.reset)
        print(console.cyan + "  remaining: " + console.yellow + "%s" % msg.remaining_locations + console.reset)
        print(console.cyan + "  state    : " + console.yellow + "%s" % msg.state + console.reset)
        print(console.cyan + "  message  : " + console.yellow + "%s" % msg.status_message + console.reset)
        print("")

    @staticmethod
    def result(msg):
        print("")
        print(console.bold + "Delivery Result" + console.reset)
        date_string = datetime.datetime.fromtimestamp(int(msg.header.stamp.to_sec())).strftime('%Y-%m-%d %H:%M:%S')
        print(console.cyan + "  timestamp: " + console.yellow + "%s" % date_string + console.reset)
        print(console.cyan + "  traversed: " + console.yellow + "%s" % msg.traversed_locations + console.reset)
        print(console.cyan + "  remaining: " + console.yellow + "%s" % msg.remaining_locations + console.reset)
        print(console.cyan + "  result   : " + console.yellow + "%s" % msg.result + console.reset)
        print(console.cyan + "  message  : " + console.yellow + "%s" % msg.error_message + console.reset)
        print("")

##############################################################################
# Classes
##############################################################################

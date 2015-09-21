#!/usr/bin/env python
#
# License: Unspecified
#
##############################################################################
# Imports
##############################################################################

import datetime
import gopher_std_msgs.msg as gopher_std_msgs
import gopher_std_msgs.srv as gopher_std_srvs
import rocon_console.console as console
import rospy
import sys

##############################################################################
# ExpressDelivery
##############################################################################


class ExpressDelivery(object):
    state_strings = {gopher_std_msgs.DeliveryFeedback.IDLE: "idle",
                     gopher_std_msgs.DeliveryFeedback.TRAVELLING: "travelling",
                     gopher_std_msgs.DeliveryFeedback.WAITING: "waiting",
                     gopher_std_msgs.DeliveryFeedback.INVALID: "invalid",
                     gopher_std_msgs.DeliveryFeedback.CANCELLED: "cancelled"
                     }

    def __init__(self):
        self.feedback_subscriber = rospy.Subscriber("/rocon/delivery/feedback", gopher_std_msgs.DeliveryFeedback, ExpressDelivery.feedback)

    def send(self, goal_locations):
        delivery_goal_request = gopher_std_srvs.DeliveryGoalRequest()
        delivery_goal_request.semantic_locations = goal_locations
        print(console.cyan + "New Goal : " + console.yellow + "%s" % delivery_goal_request.semantic_locations + console.reset)
        delivery_goal_service = rospy.ServiceProxy("/rocon/delivery/goal", gopher_std_srvs.DeliveryGoal)
        try:
            unused_delivery_goal_response = delivery_goal_service(delivery_goal_request)
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
                fetch_result = rospy.ServiceProxy('/rocon/delivery/result', gopher_std_srvs.DeliveryResult)
                response = fetch_result()
                if response.result != gopher_std_msgs.DeliveryErrorCodes.UNKNOWN:
                    ExpressDelivery.result(response)
                    break
                rate.sleep()
            except rospy.ServiceException, e:
                print "Service call failed: %s" % e
            except rospy.exceptions.ROSInterruptException:
                sys.exit()  # ros shutting down

    @staticmethod
    def feedback(msg):
        print("")
        print(console.bold + "Delivery Feedback" + console.reset)
        date_string = datetime.datetime.fromtimestamp(int(msg.header.stamp.to_sec())).strftime('%Y-%m-%d %H:%M:%S')
        print(console.cyan + "  timestamp: " + console.yellow + "%s" % date_string + console.reset)
        print(console.cyan + "  traversed: " + console.yellow + "%s" % msg.traversed_locations + console.reset)
        print(console.cyan + "  remaining: " + console.yellow + "%s" % msg.remaining_locations + console.reset)
        print(console.cyan + "  state    : " + console.yellow + "%s" % ExpressDelivery.state_strings[msg.state] + console.reset)
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

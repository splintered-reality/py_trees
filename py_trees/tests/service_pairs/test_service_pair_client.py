#!/usr/bin/env python

""" Testing the service pair client """

# enable some python3 compatibility options:
from __future__ import absolute_import, print_function, unicode_literals

import unittest
import rospy
import rocon_service_pair_msgs.msg as rocon_service_pair_msgs
import rocon_python_comms
import rostest
import unique_id
import threading


def generate_request_message():
    request = rocon_service_pair_msgs.TestiesRequest()
    request.data = "hello dude"
    return request

class TestServicePairClient(unittest.TestCase):

    def __init__(self, *args):
        super(TestServicePairClient, self).__init__(*args)
        self.testies = rocon_python_comms.ServicePairClient('testies', rocon_service_pair_msgs.TestiesPair)
        self.response_message = "I heard ya dude"
        rospy.sleep(0.5)  # rospy hack to give publishers time to setup
        self._non_blocking_msg_id = None
        self._non_blocking_msg_response = None
        self._non_blocking_with_timeout_msg_id = None
        self._non_blocking_with_timeout_msg_response = None
        self._non_blocking_event = None
        self._non_blocking_with_timeout_event = None
        
    def test_blocking_call(self):
        response = self.testies(generate_request_message())
        self.assertIsNotNone(response, "Response from the server is an invalid 'None'")
        self.assertEquals(response.data, self.response_message, "Should have received '%s' but got '%s'" % (self.response_message, response.data))

    def test_blocking_call_with_timeout(self):
        response = self.testies(generate_request_message(), timeout=rospy.Duration(3.0))
        self.assertIsNotNone(response, "Response from the server is an invalid 'None'")
        self.assertEquals(response.data, self.response_message, "Should have received '%s' but got '%s'" % (self.response_message, response.data))

    def test_non_blocking_call(self):
        self._non_blocking_event = threading.Event()
        msg_id = self.testies(generate_request_message(), callback=self.callback)
        self._non_blocking_event.wait()
        self.assertEquals(unique_id.toHexString(msg_id), unique_id.toHexString(self._non_blocking_msg_id))
        self.assertEquals(self._non_blocking_msg_response.data, self.response_message, "Should have received '%s' but got '%s'" % (self.response_message, self._non_blocking_msg_response.data))

    def test_non_blocking_call_with_timeout(self):
        self._non_blocking_with_timeout_event = threading.Event()
        msg_id = self.testies(generate_request_message(), timeout=rospy.Duration(3.0), callback=self.callback_with_timeout)
        self._non_blocking_with_timeout_event.wait()
        self.assertEquals(unique_id.toHexString(msg_id), unique_id.toHexString(self._non_blocking_with_timeout_msg_id))
        self.assertEquals(self._non_blocking_with_timeout_msg_response.data, self.response_message, "Should have received '%s' but got '%s'" % (self.response_message, self._non_blocking_with_timeout_msg_response.data))

    # def test_non_blocking_call_with_triggered_timeout(self):

    def callback(self, msg_id, msg):
        """ User callback to feed into non-blocking requests.

          @param msg_id : id of the request-response pair.
          @type uuid_msgs.UniqueID
          
          @param msg : message response received
          @type <name>Response
        """
        self._non_blocking_msg_id = msg_id
        self._non_blocking_msg_response = msg
        self._non_blocking_event.set()

    def callback_with_timeout(self, msg_id, msg):
        self._non_blocking_with_timeout_msg_id = msg_id
        self._non_blocking_with_timeout_msg_response = msg
        self._non_blocking_with_timeout_event.set()

    def error_callback(self, error_message):
        """ User callback to pick up error messages. """

if __name__ == '__main__':
    rospy.init_node("test_service_proxy")
    rostest.rosrun('rocon_python_comms',
                   'test_service_pair_client',
                   TestServicePairClient) 
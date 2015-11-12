#!/usr/bin/env python



class OpenDoor(py_trees.Behaviour):
    def __init__(self, name):
        super(OpenDoor, self).__init__(name)
        self.config = gopher_configuration.Configuration()
        self.service_client = ServicePairClient(self.config.services.door_operator, DoorControlPair)
        rospy.sleep(0.2)  # make sure the service client is initialised

    def initialise(self):
        self.response = None
        self.open_msg_sent = False
        self.sent_status_check = False
        self.got_initial_response = False
        self.got_status_response = False

    def err_cb(self, msg_id, result):
        self.response = result

    def cb(self, msg_id, msg_response):
        self.response = msg_response

    def update(self):
        if self.response is None and not self.open_msg_sent:
            req = DoorControlRequest()
            req.stamp = rospy.Time.now()
            req.cmd = DoorControlRequest.GET_STATUS
            self.response = self.service_client(req, timeout=rospy.Duration(0.3))
            self.feedback_message = "Waiting for concert to flip topics."
            return py_trees.Status.RUNNING

        rospy.loginfo("OpenDoor : Got response from concert.")

        if not self.open_msg_sent:
            req = DoorControlRequest()
            req.stamp = rospy.Time.now()
            req.cmd = DoorControlRequest.OPEN_DOOR
            self.response = None
            self.service_client(req, callback=self.cb, timeout=rospy.Duration(20), error_callback=self.err_cb)
            self.open_msg_sent = True

        if not self.got_initial_response:
            if self.response is None:
                self.feedback_message = "Sent open request to door. Waiting for response."
                return py_trees.Status.RUNNING
            elif self.response == "timeout":
                # request timed out
                self.feedback_message = "Door request timed out"
                return py_trees.Status.FAILURE
            elif self.response and self.response.cmd_resp == DoorControlResponse.IGN:
                # door is already open
                self.feedback_message = "Door was already open"
                return py_trees.Status.SUCCESS
            else:
                self.got_initial_response = True

        if not self.sent_status_check:
            req = DoorControlRequest()
            req.stamp = rospy.Time.now()
            req.cmd = DoorControlRequest.GET_STATUS
            self.response = None
            self.service_client(req, callback=self.cb, timeout=rospy.Duration(20), error_callback=self.err_cb)
            self.sent_status_check = True

        if not self.got_status_response:
            if self.response is None:
                self.feedback_message = "Sent status request to door. Waiting for response."
                return py_trees.Status.RUNNING
            elif self.response == "timeout":
                # request timed out
                self.feedback_message = "Door request timed out"
                return py_trees.Status.FAILURE
            elif self.response and self.response.status == DoorControlResponse.DOOR_OPENING:
                self.feedback_message = "Waiting for door to open"
                # reset the status check and response to get the next one
                self.sent_status_check = False
                self.response = None
                return py_trees.Status.RUNNING
            else:
                self.feedback_message = "Door is now open"
                return py_trees.Status.SUCCESS


class CloseDoor(py_trees.Behaviour):
    def __init__(self, name):
        super(CloseDoor, self).__init__(name)
        self.config = gopher_configuration.Configuration()
        self.service_client = ServicePairClient(self.config.services.door_operator, DoorControlPair)
        self.response = None
        rospy.sleep(0.2)  # make sure the service client is initialised

    def update(self):
        if self.response is None:
            req = DoorControlRequest()
            req.stamp = rospy.Time.now()
            req.cmd = DoorControlRequest.GET_STATUS
            self.response = self.service_client(req, timeout=rospy.Duration(0.3))
            self.feedback_message = "Waiting for concert to flip topics."
            return py_trees.Status.RUNNING

        rospy.loginfo("CloseDoor : Got response from concert.")

        req = DoorControlRequest()
        req.stamp = rospy.Time.now()
        req.cmd = DoorControlRequest.CLOSE_DOOR
        unused_response = self.service_client(req, rospy.Duration(0.3))

        # Don't wait for the door to finish closing
        self.feedback_message = "Sent close request to door"
        return py_trees.Status.SUCCESS

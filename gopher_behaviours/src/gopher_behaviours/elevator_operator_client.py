#
# License: Yujin
#
##############################################################################
# Description
##############################################################################

"""
.. module:: elevator_operator_client
   :platform: Unix
   :synopsis: Elevator operator related behaviours

Oh my spaghettified magnificence,
Bless my noggin with a tickle from your noodly appendages!

----

"""

##############################################################################
# Imports
##############################################################################

import elevator_interactions_msgs.msg as elevator_interactions_msgs
import gopher_configuration
import py_trees
import rocon_console.console as console
import rocon_python_comms
import rospy
import uuid

##############################################################################
# Helpers
##############################################################################


def connect_to_service(name, service_name, service_type, connect_timeout):
    '''
        connects to a service server and returns the client object

        returns None, if an exception occurs
    '''
    service_client = rocon_python_comms.ServicePairClient(
        service_name,
        service_type
    )
    try:
        if service_client.wait_for_service(rospy.Duration(connect_timeout)):
            return service_client
        else:
            return None
    except rospy.ROSInterruptException as exc:
        rospy.logerr("Behaviour [" + name + "]: ROS has been shut down while connecting to service server" +
                     " (" + str(exc) + ").")
        return None


def call_service(name, service_client, request, call_timeout):
    '''
        call the service

        returns response, if all worked fine
        returns None, if an exception occurred or the server call returned None
    '''
    response = None
    try:
        response = service_client(request, timeout=rospy.Duration(call_timeout))
    except rospy.ROSException as exc:
        rospy.logerr("Behaviour [" + name + "]: Timed out while waiting for service server's response" +
                     " (" + str(exc) + ").")
    except rospy.ROSInterruptException as exc:
        rospy.logerr("Behaviour [" + name + "]: ROS has been shut down while waiting for service server's reponse" +
                     " (" + str(exc) + ").")
    return response


##############################################################################
# Behaviours
##############################################################################

class RequestElevatorRide(py_trees.Behaviour):
    """
    Request an elevator ride
    """
    def __init__(self,
                 name="Request Elevator Ride",
                 ride_uuid=uuid.uuid4().time_low,
                 floor_pickup=0,
                 floor_dropoff=0):
        """
        :param str name: behaviour name
        """
        super(RequestElevatorRide, self).__init__(name)
        self._ride_uuid = ride_uuid
        self._floor_pickup = floor_pickup
        self._floor_dropoff = floor_dropoff
        self.config = None
        self._service_client = None
        self._request_timeout = rospy.Duration(10.0)
        self._start_time = rospy.Time(0.0)

    def setup(self, timeout):
        """

        """
        self.logger.debug("  %s [RequestElevatorRide::setup()]" % self.name)
        self.config = gopher_configuration.Configuration()
        self._service_client = connect_to_service(self.name,
                                                  self.config.services.elevator_operator,
                                                  elevator_interactions_msgs.ElevatorRidePair,
                                                  timeout)
        if self._service_client:
            return True
        else:
            rospy.logerr("Behaviour [" + self.name + "]: failed to set up ROS components.")
            return False

    def initialise(self):
        self.logger.debug("  %s [RequestElevatorRide::initialise()]" % self.name)
        self._start_time = rospy.Time.now()

    def update(self):
        self.logger.debug("  %s [RequestElevatorRide::update()]" % self.name)
        request = elevator_interactions_msgs.ElevatorRideRequest()
        request.stamp = rospy.Time.now()
        request.request_type = elevator_interactions_msgs.ElevatorRideRequest.REQUEST_RIDE
        request.ride_request.floor_pickup = self._floor_pickup
        request.ride_request.floor_dropoff = self._floor_dropoff
        request.ride_request.ride_uuid = self._ride_uuid
        # todo - switch this to non-blocking
        response = call_service(self.name, self._service_client, request, 0.25)
        if response:
            if response.ride_response.ride_uuid == self._ride_uuid:
                if response.ride_response.ride_confirmation == elevator_interactions_msgs.RideResponse.ACCEPTED:
                    self.feedback_message = "server accepted our ride request, we are #" +\
                        str(response.ride_response.waiting_line_position) + " in the queue"
                    rospy.loginfo("Behaviour [" + self.name + "]: %s" % self.feedback_message)
                    return py_trees.Status.SUCCESS
                else:
                    self.feedback_message = "server declined our ride request [response: " +\
                        str(response.ride_response.ride_confirmation) + "]"
                    rospy.logerr("Behaviour [" + self.name + "]: %s" % self.feedback_message)
                    return py_trees.Status.FAILURE
            else:
                self.feedback_message = "server is not processing our ride anymore [" +\
                    "processing ride with ID " + str(response.ride_response.ride_uuid) + "]"
                rospy.logerr("Behaviour [" + self.name + "]: %s" % self.feedback_message)
                return py_trees.Status.FAILURE
        else:
            if (rospy.Time.now() - self._start_time) > self._request_timeout:
                self.feedback_message = "no response from the server for more than %ss, giving up" % self._request_timeout
                return py_trees.Status.FAILURE
            else:
                self.feedback_message = "did not receive a response from the server, will keep trying"
                return py_trees.Status.RUNNING

    def terminate(self, new_status):
        self.logger.debug("  %s [RequestElevatorRide::terminate()][%s->%s]" % (self.name, self.status,
                                                                               new_status))


class RequestElevatorStatusUpdate(py_trees.Behaviour):
    """
    Request a status update from the elevator
    """
    def __init__(self,
                 name="Request Elevator Status Update",
                 ride_uuid=uuid.uuid4().time_low,
                 expected_result=elevator_interactions_msgs.ElevatorOperatorStatus.WAITING_FOR_BOARDING):
        """
        :param str name: behaviour name
        """
        super(RequestElevatorStatusUpdate, self).__init__(name)
        self._ride_uuid = ride_uuid
        self._expected_result = expected_result
        self.config = None
        self._service_client = None
        self._waiting_timeout = rospy.Duration(300.0)
        self._start_time = rospy.Time(0)
        self._valid_moving_states = [
            elevator_interactions_msgs.ElevatorOperatorStatus.MOVING_TO_PICKUP_FLOOR,
            elevator_interactions_msgs.ElevatorOperatorStatus.MOVING_TO_DROPOFF_FLOOR
        ]

    def setup(self, timeout):
        """

        """
        self.logger.debug("  %s [RequestElevatorStatusUpdate::setup()]" % self.name)

        self.config = gopher_configuration.Configuration()
        self._service_client = connect_to_service(self.name,
                                                  self.config.services.elevator_operator,
                                                  elevator_interactions_msgs.ElevatorRidePair,
                                                  timeout)
        if self._service_client:
            return True
        else:
            rospy.logerr("Behaviour [" + self.name + "]: Failed to set up ROS components.")
            return False

    def initialise(self):
        self.logger.debug("  %s [RequestElevatorStatusUpdate::initialise()]" % self.name)
        self._start_time = rospy.Time.now()

    def update(self):
        self.logger.debug("  %s [RequestElevatorStatusUpdate::update()]" % self.name)
        request = elevator_interactions_msgs.ElevatorRideRequest()
        request.stamp = rospy.Time.now()
        request.request_type = elevator_interactions_msgs.ElevatorRideRequest.REQUEST_ELEVATOR_STATUS_UPDATE
        response = call_service(self.name, self._service_client, request, 0.25)
        if response:
            if response.elevator_operator_status.status == self._expected_result:
                if response.elevator_operator_status.ride_uuid == self._ride_uuid:
                    self.feedback_message = "success"
                    return py_trees.Status.SUCCESS
                else:
                    # TODO handle the case where somebody else is riding and being picked up or dropped off
                    # AND we are still in the waiting queue, in that case....keep RUNNING!
                    self.feedback_message = "server is not processing our ride anymore (" +\
                        "processing ride with ID " + str(response.elevator_operator_status.ride_uuid) + ")."
                    rospy.logerr("Behaviour [" + self.name + "]: %s" % self.feedback_message)
                    return py_trees.Status.FAILURE
            else:
                # either we are moving...or we're in the waiting line
                if response.elevator_operator_status.status in self._valid_moving_states or self._ride_uuid in response.elevator_operator_status.waiting_line:
                    self.feedback_message = "received a response from the server, " +\
                        "but it's not what we are looking for (" +\
                        str(response.elevator_operator_status.status) + " != " +\
                        str(self._expected_result) + ")."
                    return py_trees.Status.RUNNING
                else:
                    self.feedback_message = "we are now no longer the elevator's overlord...[%s]" % response.elevator_operator_status.status
                    return py_trees.Status.FAILURE
        else:
            if (rospy.Time.now() - self._start_time) > self._waiting_timeout:
                self.feedback_message = "no response from the server for more than %ss, giving up" % self._request_timeout
                return py_trees.Status.FAILURE
            else:
                self.feedback_message = "did not receive a response from the server, will keep trying"
                return py_trees.Status.RUNNING

    def terminate(self, new_status):
        self.logger.debug("  %s [RequestElevatorStatusUpdate::terminate()][%s->%s]" % (self.name, self.status,
                                                                                       new_status))


class PassengerStatusUpdate(py_trees.Behaviour):
    """
    Update the elevator operator with the passenger's status
    """
    def __init__(self,
                 name="Passenger Status Update",
                 ride_uuid=uuid.uuid4().time_low,
                 status=elevator_interactions_msgs.PassengerStatus.BOARDED):
        """
        :param str name: behaviour name
        """
        super(PassengerStatusUpdate, self).__init__(name)
        self._ride_uuid = ride_uuid
        self._status = status
        self.config = None
        self._service_client = None

    def setup(self, timeout):
        """

        """
        self.logger.debug("  %s [PassengerStatusUpdate::setup()]" % self.name)
        self.config = gopher_configuration.Configuration()
        self._service_client = connect_to_service(self.name,
                                                  self.config.services.elevator_operator,
                                                  elevator_interactions_msgs.ElevatorRidePair,
                                                  timeout)
        if self._service_client:
            return True
        else:
            rospy.logerr("Behaviour [" + self.name + "]: Failed to set up ROS components.")
            return False

    def initialise(self):
        self.logger.debug("  %s [PassengerStatusUpdate::initialise()]" % self.name)

    def update(self):
        self.logger.debug("  %s [PassengerStatusUpdate::update()]" % self.name)
        request = elevator_interactions_msgs.ElevatorRideRequest()
        request.stamp = rospy.Time.now()
        request.request_type = elevator_interactions_msgs.ElevatorRideRequest.PASSENGER_STATUS_UPDATE_NOTIFICATION
        request.passenger_status.status = self._status
        request.passenger_status.ride_uuid = self._ride_uuid
        response = call_service(self.name, self._service_client, request, 0.25)
        if response:
            if (response.passenger_notification_response == elevator_interactions_msgs.ElevatorRideResponse.ACK):
                return py_trees.Status.SUCCESS
            else:
                rospy.logerr("Behaviour [" + self.name + "]: Server ignored our status update (response: " +
                             str(response.passenger_notification_response) + ").")
                return py_trees.Status.FAILURE
        else:
            self.logger.debug("Behaviour [" + self.name + "]: Did not receive a response from the server." +
                              " Will keep trying.")
            return py_trees.Status.RUNNING

    def terminate(self, new_status):
        self.logger.debug("  %s [PassengerStatusUpdate::terminate()][%s->%s]" % (self.name, self.status,
                                                                                 new_status))

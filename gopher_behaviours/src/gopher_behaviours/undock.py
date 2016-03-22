#!/usr/bin/env python

from gopher_semantics.semantics import Semantics
from py_trees.blackboard import Blackboard
import blackboard_handlers
import actionlib
import actionlib_msgs.msg as actionlib_msgs
import ar_track_alvar_msgs.msg as ar_track_alvar_msgs
import functools
import geometry_msgs.msg as geometry_msgs
import gopher_configuration
import gopher_std_msgs.msg as gopher_std_msgs
import navigation
import interactions
import py_trees
import rospy
import tf
import tf_utilities


class Undock(py_trees.Sequence):

    def __init__(self, name):
        super(Undock, self).__init__(name)
        self.gopher = gopher_configuration.Configuration()

        # wait for a transform from odom to base link
        self.add_child(navigation.WaitForTransform('Dock odom transform', 'docking_start_transform', from_frame='odom', to_frame='base_link'))
        # undock
        self.add_child(AutoUndock("Undock"))

        # Check if the dock transform has already been computed. If not, initialise it.
        check_selector = py_trees.Selector('Have dock transform?')
        check_sequence = py_trees.Sequence('Check docking variables')
        check_sequence.add_child(blackboard_handlers.CheckBlackboardVariable('Check dock transform', 'T_homebase_to_dock', expected_value=None, invert=True))
        check_sequence.add_child(CheckDockingStationMatch('Check docking station match'))

        check_selector.add_child(check_sequence)

        init_seq = py_trees.Sequence('Init dock transform')
        # get a human to drive to homebase and press the go button
        go_to_homebase = interactions.SendNotification('flash purple', message='waiting for button press to continue', led_pattern=self.gopher.led_patterns.humans_i_need_help)
        wait_button = interactions.WaitForButton('press go button at homebase', self.gopher.buttons.go)
        go_to_homebase.add_child(wait_button)
        init_seq.add_child(go_to_homebase)

        # get the odom->base link transform again, and find the transform
        # between start and end positions. Based on this, compute a position
        # which the docking procedure should go to before doing the
        # auto-docking.
        init_seq.add_child(navigation.WaitForTransform('Homebase odom transform', 'docking_end_transform', from_frame='odom', to_frame='base_link'))
        init_seq.add_child(SaveDockingLocation('Save dock location and send dslam init_pose'))

        check_selector.add_child(init_seq)

        self.add_child(check_selector)

        self.semantic_locations = Semantics(gopher_configuration.Configuration().namespaces.semantics)
        self.blackboard = py_trees.blackboard.Blackboard()

        # need to track the docking station markers so we know to re-initialise
        # when the docking station changes between delivery runs
        if not hasattr(self.blackboard, 'docking_station_markers'):
            self.blackboard.docking_station_markers = None

        if not hasattr(self.blackboard, 'previous_docking_station_markers'):
            self.blackboard.docking_station_markers = None

        if not hasattr(self.blackboard, 'homebase_transform'):
            self.blackboard.homebase = self.semantic_locations.semantic_modules['locations']['homebase']
            # assume map frame is 0,0,0 with no rotation; translation of homebase is
            # the position of the homebase specified in semantic locations. We need
            # the homebase location regardless of whether dslam is initialised.
            hb_translation = (self.blackboard.homebase.pose.x, self.blackboard.homebase.pose.y, 0)
            # quaternion specified by rotating theta radians around the yaw axis
            hb_rotation = tuple(tf.transformations.quaternion_about_axis(self.blackboard.homebase.pose.theta, (0, 0, 1)))
            self.blackboard.homebase_transform = (hb_translation, hb_rotation)

class AutoUndock(py_trees.Behaviour):

    def __init__(self, name):
        super(AutoUndock, self).__init__(name)
        self.docking_client = actionlib.SimpleActionClient(gopher_configuration.Configuration().actions.autonomous_docking,
                                                           gopher_std_msgs.AutonomousDockingAction)
        rospy.on_shutdown(functools.partial(self.stop, py_trees.Status.FAILURE))
        self.blackboard = Blackboard()
        self.connected = False
        self.goal_sent = False

    def initialise(self):
        self.connected = False
        self.goal_sent = False

    def update(self):
        self.connected = self.docking_client.wait_for_server(rospy.Duration(0.5))
        if not self.connected:
            rospy.logerr("Undock : could not connect to autonomous docking server.")
            self.feedback_message = "Action client failed to connect"
            return py_trees.Status.FAILURE

        if not self.goal_sent:
            goal = gopher_std_msgs.AutonomousDockingGoal()
            goal.command = gopher_std_msgs.AutonomousDockingGoal.UNDOCK
            self.docking_client.send_goal(goal)
            self.goal_sent = True

        result = self.docking_client.get_result()
        if result:
            if result.value == gopher_std_msgs.AutonomousDockingResult.SUCCESS:
                rospy.logdebug("Undock : " + result.message)
                self.feedback_message = result.message
                self.blackboard.previous_docking_station_markers = self.blackboard.docking_station_markers
                self.blackboard.docking_station_markers = (result.dock_marker_id_top, result.dock_marker_id_left, result.dock_marker_id_right)
                return py_trees.Status.SUCCESS
            elif result.value == gopher_std_msgs.AutonomousDockingResult.ABORTED_OBSTACLES:
                rospy.logdebug("Undock : " + result.message)
                self.feedback_message = result.message
                return py_trees.Status.FAILURE
            else:
                rospy.logdebug("Undock : got unknown result value from undocking controller")
                self.feedback_message = "Got unknown result value from undocking controller"
                return py_trees.Status.FAILURE
        else:
            rospy.logdebug("Undock : undocking in progress")
            self.feedback_message = "Undocking in progress"
            return py_trees.Status.RUNNING

    def stop(self, new_status):
        if self.goal_sent:
            action_state = self.docking_client.get_state()
            if action_state == actionlib_msgs.GoalStatus.PENDING or action_state == actionlib_msgs.GoalStatus.ACTIVE:
                self.docking_client.cancel_goal()

class CheckDockingStationMatch(py_trees.Behaviour):

    def __init__(self, name):
        super(CheckDockingStationMatch, self).__init__(name)
        self.blackboard = Blackboard()

    def update(self):
        if self.blackboard.previous_docking_station_markers == self.blackboard.docking_station_markers:
            self.feedback_message = 'docking station was the same as before'
            return py_trees.Status.SUCCESS
        else:
            self.feedback_message = 'docking station changed since last undock'
            return py_trees.Status.FAILURE

class SaveDockingLocation(py_trees.Behaviour):

    def __init__(self, name):
        super(SaveDockingLocation, self).__init__(name)
        self.blackboard = Blackboard()
        self._location_publisher = rospy.Publisher(gopher_configuration.Configuration().topics.initial_pose, geometry_msgs.PoseWithCovarianceStamped, queue_size=1)

    def update(self):
        if not hasattr(self.blackboard, 'docking_end_transform' or not hasattr(self.blackboard, 'docking_start_transform')):
            self.feedback_message = 'blackboard did not have one of the required variables docking_end_transform or docking_start_transform'
            return py_trees.Status.FAILURE

        try:
            self.blackboard.T_homebase_to_dock = tf_utilities.inverse_times(self.blackboard.docking_end_transform,
                                                                            self.blackboard.docking_start_transform)

            self.feedback_message = 'saved docking location transform'
        except Exception as e:
            self.feedback_message = 'saving failed: {0}'.format(e)
            return py_trees.Status.FAILURE

        try:
            pose = geometry_msgs.PoseWithCovarianceStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "map"
            pose.pose.pose = tf_utilities.transform_to_pose((self.blackboard.homebase_transform))
            self._location_publisher.publish(pose)
        except Exception as e:
            self.feedback_message = 'failed to send pose init: {0}'.format(e)
            return py_trees.Status.FAILURE

        self.blackboard.last_action_unparked = False
        return py_trees.Status.SUCCESS

if __name__ == '__main__':
    # assumes simulation is running
    rospy.init_node("undock_test")

    undock = Undock("undock")

    blackboard = Blackboard()

    tree = py_trees.ROSBehaviourTree(undock)
    tree.tick_tock(500)

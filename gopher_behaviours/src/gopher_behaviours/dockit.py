#!/usr/bin/env python

from .navigation import SimpleMotion
from actionlib_msgs.msg import GoalStatus
from gopher_semantics.docking_stations import DockingStations
from gopher_semantics.semantics import Semantics
import actionlib
import ar_track_alvar_msgs.msg as ar_track_alvar_msgs
import geometry_msgs.msg as geometry_msgs
import gopher_configuration
import gopher_std_msgs.msg as gopher_std_msgs
import py_trees
import rospy
import std_msgs.msg as std_msgs
import tf
import tf_utilities

##############################################################################
# Behaviours
##############################################################################


class Dock(py_trees.Behaviour):
    """
    Engage the docking behaviour
    """
    def __init__(self, name):
        super(Dock, self).__init__(name)
        self.config = gopher_configuration.Configuration()
        self.blackboard = py_trees.Blackboard()
        self.semantic_locations = Semantics(self.config.namespaces.semantics)
        self.tf_listener = tf.TransformListener()
        self.motion = SimpleMotion()
        self.goal_sent = False
        self.docking_client = actionlib.SimpleActionClient(self.config.actions.autonomous_docking,
                                                           gopher_std_msgs.AutonomousDockingAction)

        # if we want to provide an interaction for docking cancellation
        # self._interrupt_sub = rospy.Subscriber(self.config.buttons.stop, std_msgs.Empty, self.interrupt_cb)
        self._honk_publisher = rospy.Publisher(self.config.sounds.honk, std_msgs.Empty, queue_size=1)

    def initialise(self):

        if self.status == py_trees.Status.FAILURE:
            return

        self.interrupted = False  # button pressed to interrupt docking procedure?
        self.goal_sent = False  # docking goal sent?
        self.not_undocked = False
        self.got_transform = False
        self.timeout = rospy.Time.now() + rospy.Duration(3)

        # if the homebase to dock transform is unset, we didn't do a docking
        # action - this could mean that the run was started without doing any
        # undock or unpark action
        if not hasattr(self.blackboard, 'T_homebase_to_dock') or self.blackboard.T_homebase_to_dock is None:
            self.not_undocked = True
            return

        self.homebase = self.semantic_locations.semantic_modules['locations']['homebase']
        self.hb_translation = (self.homebase.pose.x, self.homebase.pose.y, 0)
        self.hb_rotation = tuple(tf.transformations.quaternion_about_axis(self.homebase.pose.theta, (0, 0, 1)))

        if self._honk_publisher:
            self._honk_publisher.publish(std_msgs.Empty())

    def cancel_docking(self, msg):
        rospy.logdebug("Cancelling any current docking action")
        self.interrupted = True
        self.docking_client.cancel_goal()
        self.motion.stop()

    def update(self):
        if self.interrupted:
            rospy.logdebug("Dock : interrupted")
            self.feedback_message = "docking was interrupted by button press"
            return py_trees.Status.FAILURE

        if self.not_undocked:
            rospy.logdebug("Dock : Transform to dock was unset - no undock action was performed")
            self.feedback_message = "Transform to dock was unset - no undock action was performed"
            return py_trees.Status.FAILURE

        if self.timeout > rospy.Time.now() and not self.got_transform:
            rospy.logdebug("Dock : waiting for transform from map to base_link")
            try:
                start_transform = self.tf_listener.lookupTransform("map", "base_link", rospy.Time(0))
            except tf.Exception:
                self.feedback_message = "waiting for transform from map to base_link"
                return py_trees.Status.RUNNING

            rospy.logdebug("Dock : got transform")
            T_hb_to_current = tf_utilities.inverse_times((self.hb_translation, self.hb_rotation), start_transform)
            T_hb_to_dock = self.blackboard.T_homebase_to_dock

            rospy.logdebug("Dock : transform from homebase to current location")
            rospy.logdebug(tf_utilities.human_transform(T_hb_to_current))
            rospy.logdebug("Dock : transform from homebase to docking location")
            rospy.logdebug(tf_utilities.human_transform(T_hb_to_dock))

            # create transformstamped objects for the transformations from homebase to the two locations
            t = tf.Transformer(True, rospy.Duration(10))
            cur = tf_utilities.transform_to_transform_stamped(T_hb_to_current)
            cur.header.frame_id = "homebase"
            cur.child_frame_id = "current"
            t.setTransform(cur)

            dock = tf_utilities.transform_to_transform_stamped(T_hb_to_dock)
            dock.header.frame_id = "homebase"
            dock.child_frame_id = "docking"
            t.setTransform(dock)

            T_current_to_dock = t.lookupTransform("current", "docking", rospy.Time(0))

            rospy.logdebug("Dock : transform from current location to dock")
            rospy.logdebug(tf_utilities.human_transform(T_current_to_dock))

            self.motion.execute("rotate", tf_utilities.transform_bearing(T_current_to_dock))
            rospy.logdebug("Dock : rotating to face docking station")
            self.feedback_message = "rotating to face docking station"
            self.got_transform = True
            return py_trees.Status.RUNNING

        if not self.got_transform:
            rospy.logdebug("Dock : could not get transform from map to base link")
            self.feedback_message = "Could not get transform from map to base link"
            return py_trees.Status.FAILURE

        # running if the rotation motion is still executing
        if not self.motion.complete():
            rospy.logdebug("Dock : rotating to face docking station")
            self.feedback_message = "Rotating to face docking station"
            return py_trees.Status.RUNNING

        # if the rotation motion fails, the behaviour fails
        if not self.motion.success():
            rospy.logerr("Dock : failed to rotate to docking station")
            self.feedback_message = "Failed to rotate to docking station"
            return py_trees.Status.FAILURE
        else:
            if not self.goal_sent:
                self.connected = self.docking_client.wait_for_server(rospy.Duration(0.5))
                if not self.connected:
                    rospy.logerr("Dock : could not connect to autonomous docking server.")
                    self.feedback_message = "Action client failed to connect"
                    return py_trees.Status.INVALID

                # the rotation succeeded, now send the docking goal
                goal = gopher_std_msgs.AutonomousDockingGoal()
                goal.command = gopher_std_msgs.AutonomousDockingGoal.DOCK
                self.docking_client.send_goal(goal)
                self.goal_sent = True

        # at this point, we have sent the docking goal - check whether it's
        # succeeded or failed, or if it's still running.
        result = self.docking_client.get_result()
        if result:
            if result.value == gopher_std_msgs.AutonomousDockingResult.SUCCESS:
                rospy.logdebug("Dock : " + result.message)
                self.feedback_message = result.message
                return py_trees.Status.SUCCESS
            elif result.value == gopher_std_msgs.AutonomousDockingResult.ABORTED_OBSTACLES:
                rospy.logdebug("Dock : " + result.message)
                self.feedback_message = result.message
                return py_trees.Status.FAILURE
            else:
                rospy.logdebug("Dock : got unknown result value from docking controller")
                self.feedback_message = "Got unknown result value from docking controller"
                return py_trees.Status.FAILURE
        else:
            rospy.logdebug("Dock : docking in progress")
            self.feedback_message = "Docking in progress"
            return py_trees.Status.RUNNING

    def stop(self, new_status):
        self.motion.stop()
        if self.goal_sent:
            self.docking_client.cancel_goal()
        # reset the homebase to dock transform to none so we can use it as a
        # flag to indicate that there was no undock performed
        if new_status == py_trees.Status.FAILURE:
            self.blackboard.T_homebase_to_dock = None


class Undock(py_trees.Behaviour):
    """
    Leave the docking bay. This is typically a specialised maneuvre so that
    the robot can begin localising.
    """
    def __init__(self, name):
        super(Undock, self).__init__(name)
        self.config = gopher_configuration.Configuration()
        self.action_client = actionlib.SimpleActionClient(self.config.actions.autonomous_docking,
                                                          gopher_std_msgs.AutonomousDockingAction)
        self._location_publisher = rospy.Publisher(self.config.topics.initial_pose,
                                                   geometry_msgs.PoseWithCovarianceStamped, queue_size=1)
        self._honk_publisher = None
        self.blackboard = py_trees.Blackboard()
        self.docking_stations = DockingStations()
        self.semantic_locations = Semantics(self.config.namespaces.semantics)
        self.sent_goal = False
        self.got_markers = False
        self.blackboard.T_homebase_to_dock = None

        try:
            if rospy.get_param("~enable_honks") and rospy.get_param("~undocking_honk"):
                honk_topic = rospy.get_param("~undocking_honk")
                self._honk_publisher = rospy.Publisher("/gopher/commands/sounds/" + honk_topic, std_msgs.Empty, queue_size=1)
        except KeyError:
            rospy.logwarn("Gopher Deliveries : Could not find param to initialise honks.")
            pass

    def initialise(self):
        self.homebase = self.semantic_locations.semantic_modules['locations']['homebase']
        self.blackboard.parked = False
        self.sent_goal = False
        self.got_markers = False
        self.ar_markers_short = None
        self._ar_short_subscriber = rospy.Subscriber("ar_pose_marker_short_range", ar_track_alvar_msgs.AlvarMarkers, self.ar_cb_short)

        # wait a while for ar messages to come in
        self.timeout = rospy.Time.now() + rospy.Duration(3)

    def ar_cb_short(self, msg):
        # can have multiple short range markers
        self.ar_markers_short = [marker.id for marker in msg.markers]
        if self.ar_markers_short:
            self.got_markers = True

    def update(self):
        self.logger.debug("  %s [Undock::update()]" % self.name)
        # need to wait for AR marker information to come through

        if not self.got_markers:
            if rospy.Time.now() > self.timeout:
                rospy.logerr("Undock : could not find AR marker information")
                self.feedback_message = "did not get any AR marker information"
                return py_trees.Status.FAILURE
            else:
                rospy.logdebug("Undock : waiting for AR marker information")
                self.feedback_message = "waiting for AR marker information"
                return py_trees.Status.RUNNING

        if not self.sent_goal:
            self.blackboard.dock_ar_markers_small = self.ar_markers_short

            # retrieve information about this docking station
            stations = self.docking_stations.find_docking_stations_with_ar_marker_id(id_list=self.ar_markers_short)
            ds = None
            for station in stations.values():
                if (station.left_id == self.ar_markers_short[0] or station.left_id == self.ar_markers_short[1])\
                   and (station.right_id == self.ar_markers_short[0] or station.right_id == self.ar_markers_short[1]):
                    ds = station
                    break

            if ds is None:
                rospy.logerr("Undock : could not find docking station in semantic locations with visible markers")
                self.feedback_message = "could not find docking station in semantic locations with the visible markers"
                return py_trees.Status.FAILURE

            # initialise a transform with the pose of the station
            ds_translation = (ds.pose.x, ds.pose.y, 0)
            # quaternion specified by rotating theta radians around the yaw axis
            ds_rotation = tuple(tf.transformations.quaternion_about_axis(ds.pose.theta, (0, 0, 1)))
            hb_translation = (self.homebase.pose.x, self.homebase.pose.y, 0)
            hb_rotation = tuple(tf.transformations.quaternion_about_axis(self.homebase.pose.theta, (0, 0, 1)))

            # publish that information to initialise navigation, and save for the
            # docking procedure
            self.blackboard.T_homebase_to_dock = tf_utilities.inverse_times((hb_translation, hb_rotation), (ds_translation, ds_rotation))
            pose = geometry_msgs.PoseWithCovarianceStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "map"
            pose.pose.pose = tf_utilities.transform_to_pose((ds_translation, ds_rotation))
            self._location_publisher.publish(pose)
            rospy.sleep(1)  # let things initialise once the pose is sent
            rospy.logdebug("Undock : sent initial pose to initialise navigation")
            self.connected = self.action_client.wait_for_server(rospy.Duration(0.5))
            if not self.connected:
                rospy.logerr("Undock : could not connect to autonomous docking server.")
                self.feedback_message = "Action client failed to connect"
                return py_trees.Status.INVALID
            else:
                goal = gopher_std_msgs.AutonomousDockingGoal()
                goal.command = gopher_std_msgs.AutonomousDockingGoal.UNDOCK
                self.action_client.send_goal(goal)
                self.sent_goal = True

            if self._honk_publisher:
                self._honk_publisher.publish(std_msgs.Empty())

        result = self.action_client.get_result()
        if result:
            if result.value == gopher_std_msgs.AutonomousDockingResult.SUCCESS:
                rospy.logdebug("Undock : " + result.message)
                self.feedback_message = result.message
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
        if self.sent_goal:
            action_state = self.action_client.get_state()
            if action_state == GoalStatus.PENDING or action_state == GoalStatus.ACTIVE:
                self.action_client.cancel_goal()

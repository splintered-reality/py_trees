#!/usr/bin/env python
import rospy
import py_trees
import actionlib
import tf
import tf_utilities
from navigation import SimpleMotion
import gopher_configuration
from gopher_semantics.semantics import Semantics
import functools
from gopher_behaviours.time import Pause
from py_trees.blackboard import Blackboard
import gopher_std_msgs.msg as gopher_std_msgs
import actionlib_msgs.msg as actionlib_msgs


class Dock(py_trees.Sequence):

    def __init__(self, name):
        super(Dock, self).__init__(name)
        self.add_child(py_trees.CheckBlackboardVariable('check docked', 'T_homebase_to_dock', expected_value=None, invert=True))
        self.add_child(GetTransforms('get transforms'))
        # motion to dock blackboard variable initialised by get transforms
        self.add_child(DoMotion('rotate to dock', 'motion_to_dock'))
        self.add_child(Pause('pause', 2))
        self.add_child(AutoDock("dock"))

        self.blackboard = py_trees.blackboard.Blackboard()

        if not hasattr(self.blackboard, 'homebase_transform'):
            self.semantic_locations = Semantics(gopher_configuration.Configuration().namespaces.semantics)
            self.blackboard.homebase = self.semantic_locations.semantic_modules['locations']['homebase']
            # assume map frame is 0,0,0 with no rotation; translation of homebase is
            # the position of the homebase specified in semantic locations. We need
            # the homebase location regardless of whether dslam is initialised.
            hb_translation = (self.blackboard.homebase.pose.x, self.blackboard.homebase.pose.y, 0)
            # quaternion specified by rotating theta radians around the yaw axis
            hb_rotation = tuple(tf.transformations.quaternion_about_axis(self.blackboard.homebase.pose.theta, (0, 0, 1)))
            self.blackboard.homebase_transform = (hb_translation, hb_rotation)

class GetTransforms(py_trees.Behaviour):

    def __init__(self, name):
        super(GetTransforms, self).__init__(name)
        self.blackboard = Blackboard()
        self.tf_listener = tf.TransformListener()

    def initialise(self):
        self.timeout = rospy.Time.now() + rospy.Duration(3)

    def update(self):
        if self.timeout > rospy.Time.now():
            rospy.logdebug("Dock : waiting for transform from map to base_link")
            try:
                start_transform = self.tf_listener.lookupTransform("map", "base_link", rospy.Time(0))
            except tf.Exception:
                self.feedback_message = "waiting for transform from map to base_link"
                return py_trees.Status.RUNNING

            rospy.logdebug("Dock : got transform")
            T_hb_to_current = tf_utilities.inverse_times(self.blackboard.homebase_transform, start_transform)
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

            self.blackboard.motion_to_dock = ("rotate", tf_utilities.transform_bearing(T_current_to_dock))
            return py_trees.Status.SUCCESS
        else:
            return py_trees.Status.FAILURE

        def stop(self, new_status):
            # reset the homebase to dock transform to none so we can use it as a
            # flag to indicate that there was no undock performed
            if new_status == py_trees.Status.FAILURE:
                self.blackboard.T_homebase_to_dock = None

class DoMotion(py_trees.Behaviour):

    def __init__(self, name, blackboard_attr):
        """:param blackboard_attr: variable in the blackboard which contains information
            about the motion to execute, in the form of a tuple with the motion type
            followed by the motion distance.

        """
        super(DoMotion, self).__init__(name)
        self.blackboard = py_trees.blackboard.Blackboard()
        self.attr = blackboard_attr
        self.motion = SimpleMotion()
        rospy.on_shutdown(functools.partial(self.stop, py_trees.Status.FAILURE))

    def initialise(self):
        motion_def = getattr(self.blackboard, self.attr)
        self.motion.execute(motion_def[0], motion_def[1])

    def update(self):
        # if the motion isn't complete, we're running
        if not self.motion.complete():
            self.feedback_message = "waiting for motion to complete"
            return py_trees.Status.RUNNING
        else:
            # Otherwise, the motion is finished. If either of the motions fail,
            # the behaviour fails.
            if not self.motion.success():
                self.feedback_message = "motion failed"
                return py_trees.Status.FAILURE
            else:
                self.feedback_message = "motion successful"
                return py_trees.Status.SUCCESS

    def stop(self, new_status):
        self.motion.stop()
        # reset the homebase to dock transform to none so we can use it as a
        # flag to indicate that there was no undock performed
        if new_status == py_trees.Status.FAILURE:
            self.blackboard.T_homebase_to_dock = None

class AutoDock(py_trees.Behaviour):

    def __init__(self, name):
        super(AutoDock, self).__init__(name)
        self.docking_client = actionlib.SimpleActionClient(gopher_configuration.Configuration().actions.autonomous_docking,
                                                           gopher_std_msgs.AutonomousDockingAction)
        rospy.on_shutdown(functools.partial(self.stop, py_trees.Status.FAILURE))
        self.goal_sent = False
        self.blackboard = Blackboard()

    def initialise(self):
        self.goal_sent = False

    def update(self):
        self.connected = self.docking_client.wait_for_server(rospy.Duration(0.5))
        if not self.connected:
            rospy.logerr("Dock : could not connect to autonomous docking server.")
            self.feedback_message = "Action client failed to connect"
            return py_trees.Status.FAILURE

        if not self.goal_sent:
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
        # client_state = self.docking_client.get_state()
        # if self.goal_sent and client_state != actionlib_msgs.GoalStatus.SUCCEEDED and client_state != actionlib_msgs.GoalStatus.ABORTED:
        self.docking_client.cancel_goal()
        # reset the homebase to dock transform to none so we can use it as a
        # flag to indicate that there was no undock performed
        if new_status == py_trees.Status.FAILURE or new_status == py_trees.Status.INVALID:
            self.blackboard.T_homebase_to_dock = None

if __name__ == '__main__':
    # assumes simulation is running
    rospy.init_node("dock_test")

    dock = Dock("dock")

    blackboard = Blackboard()
    # initialise a transform with the pose of the station
    ds_translation = (1, 1, 0)
    # quaternion specified by rotating theta radians around the yaw axis
    ds_rotation = tuple(tf.transformations.quaternion_about_axis(0.2, (0, 0, 1)))
    hb_translation = (0, 0, 0)
    hb_rotation = tuple(tf.transformations.quaternion_about_axis(0, (0, 0, 1)))

    blackboard.T_homebase_to_dock = tf_utilities.inverse_times((hb_translation, hb_rotation), (ds_translation, ds_rotation))

    tree = py_trees.ROSBehaviourTree(dock)
    tree.tick_tock(500)

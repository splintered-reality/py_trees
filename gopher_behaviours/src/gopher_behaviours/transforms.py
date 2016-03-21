#
# License: Yujin
#
##############################################################################
# Description
##############################################################################

"""
.. module:: time
   :platform: Unix
   :synopsis: Ros time related behaviours.

Oh my spaghettified magnificence,
Bless my noggin with a tickle from your noodly appendages!

----

"""

##############################################################################
# Imports
##############################################################################

import py_trees
import rospy
import tf

##############################################################################
# Behaviours
##############################################################################


class WaitForTransform(py_trees.Behaviour):
    """Wait for a transform from the tf tree to become available
    """
    def __init__(self, name, blackboard_attr_name, from_frame, to_frame, timeout=5):
        """
        :param name: name of the behaviour
        :param blackboard_attr_name: attribute in the blackboard in which to save the retrieved transform
        :param from_frame: transform computed from this frame
        :param to_frame: transform from the from_frame to this frame
        :param timeout: time in seconds to wait before timing out
        """
        super(WaitForTransform, self).__init__(name)

        self.blackboard = py_trees.blackboard.Blackboard()
        self.lookup_attempts = 0
        self.attr_name = blackboard_attr_name
        self.from_frame = from_frame
        self.to_frame = to_frame
        self.timeout = rospy.Time.now() + rospy.Duration(timeout)

    def initialise(self):
        self.tf_listener = tf.TransformListener()

    def update(self):
        # if dslam is initialised, the map->base_link transform gives us the
        # actual position of the robot, i.e. the position of the parking
        # spot. Save this as the starting position
        rospy.logdebug("waiting for transform from {0} to {1}".format(self.from_frame, self.to_frame))
        try:
            setattr(self.blackboard, self.attr_name, self.tf_listener.lookupTransform(self.from_frame, self.to_frame, rospy.Time(0)))
            self.transform_setup = True
            rospy.logdebug("got {0} transform".format(self.from_frame))
            return py_trees.Status.SUCCESS
        except tf.Exception:
            rospy.logdebug("didn't get {0} transform".format(self.from_frame))
            self.lookup_attempts = 1
            self.feedback_message = "waiting for transform from {0} to {1}".format(self.from_frame, self.to_frame)
            return py_trees.Status.RUNNING

        if self.lookup_attempts >= 4:
            rospy.logerr("failed to get transform from {0} to {1}".format(self.from_frame, self.to_frame))
            self.feedback_message = "failed to get transform from {0} to {1}".format(self.from_frame, self.to_frame)
            return py_trees.Status.FAILURE
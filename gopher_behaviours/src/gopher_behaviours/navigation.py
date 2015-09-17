#
# License: Yujin
#
##############################################################################
# Description
##############################################################################

"""
.. module:: navigation
   :platform: Unix
   :synopsis: Navigation related behaviours

Oh my spaghettified magnificence,
Bless my noggin with a tickle from your noodly appendages!

----

"""

##############################################################################
# Imports
##############################################################################

import geometry_msgs.msg as geometry_msgs
import os
import py_trees
import rospy
import std_msgs.msg as std_msgs
from . import utilities

##############################################################################
# Behaviours
##############################################################################


class InitPose(py_trees.Behaviour):
    """
    Init the pose. Usually used in conjunction with switching maps.
    """
    def __init__(self, name, initial_pose, topic_name):
        """
        He is a mere noodly appendage - don't expect him to be smart and check if
        the pose matches the semantics, or if the topics actually connect.

        A pastafarian at a higher level should take care of that before construction.

        :param geometry_msgs.PoseStamped initial_pose: where to init after loading the map.
        :param str topic_name:
        """
        super(SwitchMap, self).__init__(name)
        self.initial_pose = initial_pose
        self.publisher = rospy.Publisher(topic_name, geometry_msgs.PoseStamped, queue_size=1)

    def update(self):
        rospy.loginfo("Gopher Deliveries : initialising the pose {x: %s, y: %s, theta: %s}" % (self.initial_pose.pose.x, self.initial_pose.pose.y, self.initial_pose.pose.theta))
        self.publisher.publish(geometry_msgs.PoseStamped())
        return py_trees.Status.SUCCESS


class SwitchMap(py_trees.Behaviour):
    """
    Switch the map. This doesn't check semantics for anything, some component
    should do that before getting to this behaviour and feed him with the
    required information to execute. This keeps the behaviour a bit simpler.

    Remember : This behaviour assumes input values are semantically correct!
               Do your checking before init'ing!
    """
    def __init__(self, name, map_filename, topic_name):
        """
        He is a mere noodly appendage - don't expect him to be smart and check if
        the map exists in semantics, or on the filesystem, or if the topics
        actually connect.

        A pastafarian at a higher level should take care of that before construction.

        :param str map_filename: full pathname to the dslam map.
        :param str topic_name:
        """
        super(SwitchMap, self).__init__(name)
        self.dslam_map = std_msgs.String(map_filename)
        self.publisher = rospy.Publisher(topic_name, std_msgs.String, queue_size=1)

    def update(self):
        rospy.loginfo("Gopher Deliveries : switching map [%s]" % self.dslam_map.data)
        self.publisher.publish(self.dslam_map)
        return py_trees.Status.SUCCESS

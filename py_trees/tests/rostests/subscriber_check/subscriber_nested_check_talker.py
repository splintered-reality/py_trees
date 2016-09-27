#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/stonier/py_trees_suite/license/LICENSE
#
##############################################################################
# Documentation
##############################################################################
"""
Pose Talker node to assist with py_trees demo programs.
"""
##############################################################################
# Imports
##############################################################################

import rospy
from geometry_msgs.msg import Pose

##############################################################################
# Main
##############################################################################

if __name__ == '__main__':
    rospy.init_node('pose_talker')

    topic_name = rospy.get_param("~topic_name", "pose_chatter")

    # Make a dummy Pose msg for breaking the ice
    utterance_pose = Pose()

    utterance_pose.position.x = 1.0
    utterance_pose.position.y = 2.0
    utterance_pose.position.z = 3.0

    utterance_pose.orientation.x = 0.0
    utterance_pose.orientation.y = 0.0
    utterance_pose.orientation.z = 0.0
    utterance_pose.orientation.w = 1.0

    # publish the utterance in pose sentence repeatedly
    pub = rospy.Publisher(topic_name, Pose, queue_size=10, latch=True)
    while not rospy.is_shutdown():
        pub.publish(utterance_pose)
        rospy.loginfo(utterance_pose)
        rospy.sleep(2)

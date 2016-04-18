#!/usr/bin/env python
#
# License: Yujin
#
##############################################################################
# Description
##############################################################################

"""
.. module:: chalk_and_talk
   :platform: Unix
   :synopsis: Collecting asynchronous information on the blackboard

Oh my spaghettified magnificence,
Bless my noggin with a tickle from your noodly appendages!
"""

##############################################################################
# Imports
##############################################################################

import geometry_msgs.msg as geometry_msgs
import gopher_configuration
import nav_msgs.msg as nav_msgs
import std_msgs.msg as std_msgs
import py_trees

from . import battery
from . import interactions

##############################################################################
# Methods
##############################################################################


def create_gopher_handlers():
    """
    Behaviours that all group together to do data collection for gophers.
    This includes button events and common topics like battery, odom and pose.
    """
    gopher = gopher_configuration.Configuration(fallback_to_defaults=True)
    button_event_handler = interactions.create_button_event_handler()
    battery_to_blackboard = battery.ToBlackboard(name="Battery2BB", topic_name=gopher.topics.battery)

    topics_to_blackboard = py_trees.composites.Sequence(name="Topics2BB")
    topics_to_blackboard.blackbox_level = py_trees.common.BlackBoxLevel.DETAIL

    odom_to_blackboard = py_trees.subscribers.ToBlackboard(
        name="Odom2BB",
        topic_name=gopher.topics.odom,
        topic_type=nav_msgs.Odometry,
        blackboard_variables={"odom": None},
        clearing_policy=py_trees.common.ClearingPolicy.NEVER
    )
    pose_to_blackboard = py_trees.subscribers.ToBlackboard(
        name="Pose2BB",
        topic_name=gopher.topics.pose,
        topic_type=geometry_msgs.PoseWithCovarianceStamped,
        blackboard_variables={"pose": None},
        clearing_policy=py_trees.common.ClearingPolicy.NEVER
    )
    world_to_blackboard = py_trees.subscribers.ToBlackboard(
        name="World2BB",
        topic_name=gopher.topics.world,
        topic_type=std_msgs.String,
        blackboard_variables={"world": None},
        clearing_policy=py_trees.common.ClearingPolicy.NEVER
    )

    topics_to_blackboard.add_child(battery_to_blackboard)
    topics_to_blackboard.add_child(odom_to_blackboard)
    topics_to_blackboard.add_child(pose_to_blackboard)
    topics_to_blackboard.add_child(world_to_blackboard)

    return (button_event_handler, topics_to_blackboard)

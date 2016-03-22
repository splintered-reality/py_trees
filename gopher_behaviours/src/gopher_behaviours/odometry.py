#
# License: Yujin
#
##############################################################################
# Description
##############################################################################

"""
.. module:: odometry
   :platform: Unix
   :synopsis: Helpers for ros odometry

Oh my spaghettified magnificence,
Bless my noggin with a tickle from your noodly appendages!
----

"""

##############################################################################
# Imports
##############################################################################

import gopher_configuration
import nav_msgs.msg as nav_msgs
import py_trees

##############################################################################
# Behaviours
##############################################################################


def create_odom_pose_to_blackboard_behaviour(
    name="OdomToBlackboard",
    blackboard_variable_name="odom"
):
    """
    Hooks up a subscriber and transfers the odometry topic to the blackboard.

    :param str name: behaviour name
    :param str blackboard_variable_name: name to write the message to
    :returns: the behaviour
    :rtype: subscribers.CheckSubscriberVariable
    """
    gopher = gopher_configuration.Configuration(fallback_to_defaults=True)

    behaviour = py_trees.SubscriberToBlackboard(
        name,
        topic_name=gopher.topics.odom,
        topic_type=nav_msgs.Odometry,
        blackboard_variable_name=blackboard_variable_name
    )
    return behaviour

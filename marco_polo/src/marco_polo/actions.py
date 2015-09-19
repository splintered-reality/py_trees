#
# License: Yujin
#
##############################################################################
# Description
##############################################################################

"""
.. module:: actions
   :platform: Unix
   :synopsis: Specific action hero mechanisms enabling teleports.

Oh my spaghettified magnificence,
Bless my noggin with a tickle from your noodly appendages!

----

"""

##############################################################################
# Imports
##############################################################################

import rospy

##############################################################################
# Imports
##############################################################################


class SwitchMap(object):
    def __init__(self, topic_name):
        pass

    def execute(self):
        rospy.loginfo("Executing map switching")


class InitPose(object):
    def __init__(self, topic_name):
        pass

    def execute(self, pose):
        """
        :param geometry_msgs/PoseWithCovarianceStamped pose: pose to initialise at.
        """
        rospy.loginfo("Executing init pose command")

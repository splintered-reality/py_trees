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

##############################################################################
# Behaviours
##############################################################################


class Pause(py_trees.Behaviour):
    """
    Does nothing until the specified timeout is reached, the returns SUCCESS
    """
    def __init__(self, name, timeout):
        """
        :param float timeout: number of seconds to pause for.
        """
        super(Pause, self).__init__(name)
        self.duration = rospy.Duration(timeout)
        self.start_time = None
        self.finish_time = None

    def initialise(self):
        self.start_time = rospy.get_rostime()
        self.finish_time = self.start_time + self.duration

    def update(self):
        return py_trees.Status.SUCCESS if rospy.get_rostime() > self.finish_time else py_trees.Status.RUNNING


class Timeout(py_trees.Behaviour):
    """
    Does nothing until the specified timeout is reached, then returns FAILURE.
    """
    def __init__(self, name, timeout):
        """
        :param float timeout: number of seconds to pause for.
        """
        super(Timeout, self).__init__(name)
        self.duration = rospy.Duration(timeout)
        self.start_time = None
        self.finish_time = None

    def initialise(self):
        self.start_time = rospy.get_rostime()
        self.finish_time = self.start_time + self.duration

    def update(self):
        return py_trees.Status.FAILURE if rospy.get_rostime() > self.finish_time else py_trees.Status.RUNNING

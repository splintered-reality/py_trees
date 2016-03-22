#
# License: Yujin
#
##############################################################################
# Description
##############################################################################

"""
.. module:: time
   :platform: Unix
   :synopsis: Time related behaviours.

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
    Does nothing until the specified timeout is reached, then returns SUCCESS
    """
    def __init__(self, name="Pause", timeout=1.0):
        """
        Prepare the behaviour

        :param string name this behaviour's name
        :param float timeout the amount of time to pause for; set to zero to pause indefinitely
        """
        super(Pause, self).__init__(name)
        self.duration = rospy.Duration(timeout)
        self.start_time = None
        self.finish_time = None

    def initialise(self):
        self.start_time = rospy.get_rostime()
        self.finish_time = self.start_time + self.duration

    def update(self):
        if self.duration == rospy.Duration(0):
            return py_trees.Status.RUNNING
        else:
            return py_trees.Status.SUCCESS if rospy.get_rostime() > self.finish_time else py_trees.Status.RUNNING


@py_trees.meta.inverter
class Timeout(Pause):
    pass

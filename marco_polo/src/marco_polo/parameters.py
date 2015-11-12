#
# License: Yujin
#
##############################################################################
# Imports
##############################################################################

import rospy
import rocon_console.console as console

###############################################################################
# Functions
###############################################################################


class Parameters:
    """
    The variables of this class are default constructed from parameters on the
    ros parameter server. Each parameter is nested in the private namespace of
    the node which instantiates this class.

    :ivar map_switching_grace: the length of time to allow for map switching
    :vartype map_switching_grace: int

    .. _resource names: http://wiki.ros.org/Names#Package_Resource_Names
    """
    def __init__(self):
        # see sphinx docs above for more detailed explanations of each parameter
        self.map_switching_grace = rospy.get_param('~map_switching_grace', 15)

    def __str__(self):
        s = console.bold + "\nParameters:\n" + console.reset
        for key in sorted(self.__dict__):
            s += console.cyan + "    %s: " % key + console.yellow + "%s\n" % (self.__dict__[key] if self.__dict__[key] is not None else '-')
        s += console.reset
        return s

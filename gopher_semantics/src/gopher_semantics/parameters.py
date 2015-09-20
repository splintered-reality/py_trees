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

    :ivar semantics_parameter_namespace: root namespace for the location of all semantics on the parameter server *['/semantics']*
    :vartype semantics_parameter_namespace: str
    """
    def __init__(self, semantics_parameter_namespace=None):
        if semantics_parameter_namespace is None:
            self.semantics_parameter_namespace = rospy.get_param('~semantics_parameter_namespace', rospy.resolve_name('~'))
        else:
            self.semantics_parameter_namespace = semantics_parameter_namespace

    def __str__(self):
        s = console.bold + "\nParameters:\n" + console.reset
        for key in sorted(self.__dict__):
            s += console.cyan + "    %s: " % key + console.yellow + "%s\n" % (self.__dict__[key] if self.__dict__[key] is not None else '-')
        s += console.reset
        return s

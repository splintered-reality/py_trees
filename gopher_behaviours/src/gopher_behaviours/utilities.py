#
# License: Yujin
#
##############################################################################
# Description
##############################################################################

"""
.. module:: utilities
   :platform: Unix
   :synopsis: Tools for debugging and development.

Oh my spaghettified magnificence,
Bless my noggin with a tickle from your noodly appendages!

----

"""

##############################################################################
# Imports
##############################################################################

import os
import py_trees
import rospkg

##############################################################################
# Debugging
##############################################################################


class DebugVisitor(py_trees.trees.VisitorBase):
    def __init__(self):
        super(DebugVisitor, self).__init__()
        self.logger = py_trees.logging.get_logger("Visitor")

    def initialise(self):
        pass

    def run(self, behaviour):
        if behaviour.feedback_message:
            self.logger.debug("  %s [visited][%s][%s]" % (behaviour.name, behaviour.status, behaviour.feedback_message))
        else:
            self.logger.debug("  %s [visited][%s]" % (behaviour.name, behaviour.status))

##############################################################################
# Configuration
##############################################################################


def get_gopher_home():
    '''
      Retrieve the location of the gopher home directory.
      If it does not exist, create a new directory and return this path.

      .. todo:: move this to a common library (currently also used by gopher_customisation)

      :return: the gopher home directory (path)
      :rtype str:
    '''
    gopher_home = os.path.join(rospkg.get_ros_home(), 'gopher')
    if not os.path.isdir(gopher_home):
        os.makedirs(gopher_home)
    return os.path.join(rospkg.get_ros_home(), 'gopher')

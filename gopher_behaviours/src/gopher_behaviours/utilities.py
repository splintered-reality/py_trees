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
import rospkg

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

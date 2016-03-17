#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/yujinrobot/gopher_crazy_hospital/license/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
.. module:: common
   :platform: Unix
   :synopsis: Common variables and methods used by all elements in py_trees.

----

"""

##############################################################################
# Imports
##############################################################################

from enum import Enum
##############################################################################
# Status
##############################################################################

# """ An enumerator representing the status of a behaviour """
# Status = Enum('Status', 'SUCCESS FAILURE RUNNING INVALID')


class Status(Enum):
    """ An enumerator representing the status of a behaviour """

    SUCCESS = "SUCCESS"
    """Behaviour check has passed, or execution of its action has finished with a successful result."""
    FAILURE = "FAILURE"
    """Behaviour check has failed, or execution of its action finished with a failed result."""
    RUNNING = "RUNNING"
    """Behaviour is in the middle of executing some action, result still pending."""
    INVALID = "INVALID"
    """Behaviour is uninitialised and inactive."""

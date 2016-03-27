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
    """Behaviour is uninitialised and inactive, i.e. this is the status before first entry, and after a higher priority switch has occurred."""


# class ClearingPolicy(Enum):
#     """
#     Policy rules for behaviours to dictate when data should be cleared/reset.
#     """
#     ON_INITIALISE = 1
#     """Clear when entering the :py:meth:`~py_trees.behaviour.Behaviour.initialise` method."""
#     ON_TERMINATE = 2
#     """Clear when entering the :py:meth:`~py_trees.behaviour.Behaviour.terminate` method."""
#     ON_INVALID = 3
#     """Only when interrupted by a higher priority branch."""
#     NEVER_CLEAR = 4
#     """Never clear the data"""

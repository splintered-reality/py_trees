#!/usr/bin/env python
#
# License: Yujin
#
##############################################################################
# Description
##############################################################################

"""
.. module:: starting
   :platform: Unix
   :synopsis: Common components for starting/finishing behaviours.

Oh my spaghettified magnificence,
Bless my noggin with a tickle from your noodly appendages!

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


class StartingAction(Enum):
    """ An enumerator representing the kind of starting action that was engaged."""

    UNPARKED = "unparked"
    """Unparked"""
    UNDOCKED = "undocked"
    """Undocked"""

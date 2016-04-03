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

import enum

##############################################################################
# Status
##############################################################################

# """ An enumerator representing the status of a behaviour """
# Status = Enum('Status', 'SUCCESS FAILURE RUNNING INVALID')


class Status(enum.Enum):
    """ An enumerator representing the status of a behaviour """

    SUCCESS = "SUCCESS"
    """Behaviour check has passed, or execution of its action has finished with a successful result."""
    FAILURE = "FAILURE"
    """Behaviour check has failed, or execution of its action finished with a failed result."""
    RUNNING = "RUNNING"
    """Behaviour is in the middle of executing some action, result still pending."""
    INVALID = "INVALID"
    """Behaviour is uninitialised and inactive, i.e. this is the status before first entry, and after a higher priority switch has occurred."""


class ClearingPolicy(enum.IntEnum):
    """
    Policy rules for behaviours to dictate when data should be cleared/reset.
    Used by the :py:mod:`~py_trees.subscribers` module.
    """
    ON_INITIALISE = 1
    """Clear when entering the :py:meth:`~py_trees.behaviour.Behaviour.initialise` method."""
    ON_SUCCESS = 2
    """Clear when returning :py:data:`~py_trees.common.Status.SUCCESS`."""
    NEVER = 3
    """Never clear the data"""


class BlackBoxLevel(enum.IntEnum):
    """
    Whether a behaviour is a blackbox entity that may be considered collapsible
    (i.e. everything in its subtree will not be visualised) by
    visualisation tools.

    Blackbox levels are increasingly persistent in visualisations.

    Visualisations by default, should always collapse blackboxes that represent
    `FINE_DETAIL`.
    """
    FINE_DETAIL = 1
    """A blackbox that doesn't wish to expose the fine detail beneath unless forced."""
    DETAIL = 2
    """A blackbox that encapsulates detailed activity."""
    COMPONENT = 3
    """A blackbox that encapsulates a subgroup of functionalities as a single group."""
    BIG_PICTURE = 4
    """A blackbox that represents a big picture part of the entire tree view."""
    NOT_A_BLACKBOX = 5
    """Not a blackbox, do not ever collapse."""


class VisibilityLevel(enum.IntEnum):
    """
    Closely associated with the :py:class:`~py_trees.common.BlackBoxLevel` for a
    behaviour. This sets the visibility level to be used for visualisations.

    Visibility levels correspond to reducing levels of visibility in a visualisation.
    """
    ALL = 0
    """Do not collapse any behaviour."""
    FINE_DETAIL = BlackBoxLevel.FINE_DETAIL
    """Collapse blackboxes marked as :py:data:`~py_trees.common.BlackBoxLevel.FINE_DETAIL`."""
    DETAIL = BlackBoxLevel.DETAIL
    """Collapse blackboxes marked with :py:data:`~py_trees.common.BlackBoxLevel.DETAIL` or lower."""
    COMPONENT = BlackBoxLevel.COMPONENT
    """Collapse blackboxes marked with :py:data:`~py_trees.common.BlackBoxLevel.COMPONENT` or lower."""
    BIG_PICTURE = BlackBoxLevel.BIG_PICTURE
    """Collapse any blackbox that isn't marked with :py:data:`~py_trees.common.BlackBoxLevel.BIG_PICTURE`."""

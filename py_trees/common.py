#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/stonier/py_trees/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
Common definitions, methods and variables used by the py_trees library.
"""

##############################################################################
# Imports
##############################################################################

import enum

##############################################################################
# Status
##############################################################################


class Status(enum.Enum):
    """An enumerator representing the status of a behaviour """

    SUCCESS = "SUCCESS"
    """Behaviour check has passed, or execution of its action has finished with a successful result."""
    FAILURE = "FAILURE"
    """Behaviour check has failed, or execution of its action finished with a failed result."""
    RUNNING = "RUNNING"
    """Behaviour is in the middle of executing some action, result still pending."""
    INVALID = "INVALID"
    """Behaviour is uninitialised and inactive, i.e. this is the status before first entry, and after a higher priority switch has occurred."""


class ParallelPolicy(object):
    class Base(object):
        """
        Base class for parallel policies. Should never be used directly.
        """
        def __init__(self, synchronise=False):
            self.synchronise = synchronise

    class SuccessOnAll(Base):
        """
        Return :py:data:`~py_trees.common.Status.SUCCESS` only when each and every child returns
        :py:data:`~py_trees.common.Status.SUCCESS`.
        """
        def __init__(self, synchronise=True):
            super().__init__(synchronise=synchronise)

    class SuccessOnOne(Base):
        """
        Return :py:data:`~py_trees.common.Status.SUCCESS` so long as at least one child has :py:data:`~py_trees.common.Status.SUCCESS`
        and the remainder are :py:data:`~py_trees.common.Status.RUNNING`
        """
        def __init__(self):
            super().__init__(synchronise=False)

    class SuccessOnSelected(Base):
        """
        Retrun :py:data:`~py_trees.common.Status.SUCCESS` so long as each child in a specified list returns
        :py:data:`~py_trees.common.Status.SUCCESS`.
        """
        def __init__(self, children, synchronise=True):
            super().__init__(synchronise=synchronise)
            self.children = children


class OneShotPolicy(enum.Enum):
    """Policy rules for :py:class:`~py_trees.decorators.OneShot` (decorator) or :py:meth:`~py_trees.idioms.oneshot (idiom) oneshots."""

    ON_COMPLETION = [Status.SUCCESS, Status.FAILURE]
    """Return :py:data:`~py_trees.common.Status.SUCCESS` after the specified child/subtree reaches completion (:py:data:`~py_trees.common.Status.SUCCESS` || :py:data:`~py_trees.common.Status.FAILURE`)."""
    ON_SUCCESSFUL_COMPLETION = [Status.SUCCESS]
    """Permits the oneshot to keep trying until it's first success."""


class Name(enum.Enum):
    """
    Naming conventions.
    """
    AUTO_GENERATED = "AUTO_GENERATED"
    """:py:data:`~py_trees.common.Name.AUTO_GENERATED` leaves it to the behaviour to generate a useful, informative name."""


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
    `DETAIL`.
    """
    DETAIL = 1
    """A blackbox that encapsulates detailed activity."""
    COMPONENT = 2
    """A blackbox that encapsulates a subgroup of functionalities as a single group."""
    BIG_PICTURE = 3
    """A blackbox that represents a big picture part of the entire tree view."""
    NOT_A_BLACKBOX = 4
    """Not a blackbox, do not ever collapse."""


class VisibilityLevel(enum.IntEnum):
    """
    Closely associated with the :py:class:`~py_trees.common.BlackBoxLevel` for a
    behaviour. This sets the visibility level to be used for visualisations.

    Visibility levels correspond to reducing levels of visibility in a visualisation.
    """
    ALL = 0
    """Do not collapse any behaviour."""
    DETAIL = BlackBoxLevel.DETAIL
    """Collapse blackboxes marked with :py:data:`~py_trees.common.BlackBoxLevel.DETAIL` or lower."""
    COMPONENT = BlackBoxLevel.COMPONENT
    """Collapse blackboxes marked with :py:data:`~py_trees.common.BlackBoxLevel.COMPONENT` or lower."""
    BIG_PICTURE = BlackBoxLevel.BIG_PICTURE
    """Collapse any blackbox that isn't marked with :py:data:`~py_trees.common.BlackBoxLevel.BIG_PICTURE`."""


visibility_level_strings = ["all", "detail", "component", "big_picture"]
"""Convenient string representations to use for command line input (amongst other things)."""


def string_to_visibility_level(level):
    """
    Will convert a string to a visibility level. Note that it will quietly return ALL if
    the string is not matched to any visibility level string identifier.

    Args:
        level (str): visibility level as a string

    Returns:
        :class:`~py_trees.common.VisibilityLevel`: visibility level enum
    """
    if level == "detail":
        return VisibilityLevel.DETAIL
    elif level == "component":
        return VisibilityLevel.COMPONENT
    elif level == "big_picture":
        return VisibilityLevel.BIG_PICTURE
    else:
        return VisibilityLevel.ALL

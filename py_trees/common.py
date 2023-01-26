#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/splintered-reality/py_trees/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""Common definitions, methods and variables used by the py_trees library."""

##############################################################################
# Imports
##############################################################################

import enum
import sys
import typing

##############################################################################
# General
##############################################################################


class Name(enum.Enum):
    """Naming conventions."""

    AUTO_GENERATED = "AUTO_GENERATED"
    """Automagically generate (hopefully) something sensible.."""


class Status(enum.Enum):
    """An enumerator representing the status of a behaviour."""

    SUCCESS = "SUCCESS"
    """Behaviour check has passed, or execution of its action has finished with a successful result."""
    FAILURE = "FAILURE"
    """Behaviour check has failed, or execution of its action finished with a failed result."""
    RUNNING = "RUNNING"
    """Behaviour is in the middle of executing some action, result still pending."""
    INVALID = "INVALID"
    """Behaviour is uninitialised and/or in an inactive state, i.e. not currently being ticked."""


class Duration(enum.Enum):
    """Naming conventions."""

    INFINITE = sys.float_info.max
    """:py:data:`~py_trees.common.Duration.INFINITE` oft used for perpetually blocking operations."""
    UNTIL_THE_BATTLE_OF_ALFREDO = sys.float_info.max
    """:py:data:`~py_trees.common.Duration.UNTIL_THE_BATTLE_OF_ALFREDO` is an alias for
    :py:data:`~py_trees.common.Duration.INFINITE`."""


class Access(enum.Enum):
    """Use to distinguish types of access to, e.g. variables on a blackboard."""

    READ = "READ"
    """Read access."""
    WRITE = "WRITE"
    """Write access, implicitly also grants read access."""
    EXCLUSIVE_WRITE = "EXCLUSIVE_WRITE"
    """Exclusive lock for writing, i.e. no other writer permitted."""


##############################################################################
# Policies
##############################################################################


class ParallelPolicy(object):
    """Configurable policies for :py:class:`~py_trees.composites.Parallel` behaviours."""

    class Base(object):
        """Base class for parallel policies. Should never be used directly."""

        def __init__(self, synchronise: bool = False):
            """
            Configure the policy to be synchronised or otherwise.

            Args:
                synchronise: stop ticking of children with status
                    :py:data:`~py_trees.common.Status.SUCCESS` until the policy criteria is met
            """
            self.synchronise = synchronise

    class SuccessOnAll(Base):
        """
        Success depends on all children succeeding.

        Return :py:data:`~py_trees.common.Status.SUCCESS` only when each and every child returns
        :py:data:`~py_trees.common.Status.SUCCESS`. If synchronisation is requested, any children that
        tick with :data:`~py_trees.common.Status.SUCCESS` will be skipped on subsequent ticks until
        the policy criteria is met, or one of the children returns status :data:`~py_trees.common.Status.FAILURE`.
        """

        def __init__(self, synchronise: bool = True):
            """
            Policy configuration.

            Args:
                synchronise (:obj:`bool`): stop ticking of children with status
                    :py:data:`~py_trees.common.Status.SUCCESS` until the policy criteria is met
            """
            super().__init__(synchronise=synchronise)

    class SuccessOnOne(Base):
        """Success depends on just one child (can be any child).

        Return :py:data:`~py_trees.common.Status.SUCCESS` so long as at least one child has
        :py:data:`~py_trees.common.Status.SUCCESS`
        and the remainder are :py:data:`~py_trees.common.Status.RUNNING`
        """

        def __init__(self) -> None:
            """No configuration necessary for this policy."""
            super().__init__(synchronise=False)

    class SuccessOnSelected(Base):
        """Success depends on an explicitly selected set of children behaviours.

        Return :py:data:`~py_trees.common.Status.SUCCESS` so long as each child in a specified list returns
        :py:data:`~py_trees.common.Status.SUCCESS`. If synchronisation is requested, any children that
        tick with :data:`~py_trees.common.Status.SUCCESS` will be skipped on subsequent ticks until
        the policy criteria is met, or one of the children returns status :data:`~py_trees.common.Status.FAILURE`.
        """

        def __init__(
            self,
            # TODO should be behaviour.Behaviour, but cyclic imports -> redesign
            children: typing.List[typing.Any],
            synchronise: bool = True,
        ):
            """
            Policy configuration.

            Args:
                children: list of children to succeed on
                synchronise: stop ticking of children with status
                :py:data:`~py_trees.common.Status.SUCCESS` until the policy criteria is met
            """
            super().__init__(synchronise=synchronise)
            self.children = children


class OneShotPolicy(enum.Enum):
    """Policy rules for oneshots.

    These are used to configure both :py:class:`~py_trees.decorators.OneShot` (decorator)
    and py:meth:`~py_trees.idioms.oneshot (idiom) approaches.
    """

    ON_COMPLETION = [Status.SUCCESS, Status.FAILURE]
    """Reflect the child/subtree's status as soon as it ticks to completion (success or failure)."""
    ON_SUCCESSFUL_COMPLETION = [Status.SUCCESS]
    """Reflect the child/subtree's status only when it succeeds (failures are rerun)."""


class ClearingPolicy(enum.IntEnum):
    """Policy rules for behaviours to dictate when data should be cleared/reset."""

    ON_INITIALISE = 1
    """Clear when entering the :py:meth:`~py_trees.behaviour.Behaviour.initialise` method."""
    ON_SUCCESS = 2
    """Clear when returning :py:data:`~py_trees.common.Status.SUCCESS`."""
    NEVER = 3
    """Never clear the data"""


##############################################################################
# Blackboards
##############################################################################


# TODO: There might be a case for allowing some widening of the arguments,
# i.e. for left and right hand argument types to the ComparisonExpression
# operator to vary and/or be covariant / contravariant so different classes
# or base/derived variants can be compared.
#
# To do so however, starts to make the machinery awkward (e.g. forcing
# the user to only use the 'value' as a right-hand argument to the operator.
# If this is necessary, a different construct would be better.
#
# NB: Widening all the way to Any results in virally plaguing downstream
# with the ramifications of Any (e.g. any-return).
ComparisonV = typing.TypeVar("ComparisonV")


class ComparisonExpression(object):
    """
    Store the parameters for a univariate comparison operation.

    A univariate comparison operation compares the relationship between
    a variable and a value (e.g. x > 3).

    Args:
        variable: name of the variable to compare
        value: value to compare against
        operator: a callable comparison operator

    .. tip::
        The python `operator module`_ includes many useful comparison operations, e.g. operator.ne

    .. code::
        import operator

        check = ComparisonExpression(
            variable="foo",
            value= 5,
            operator=operator.eq
        )
        success = check.operator(blackboard[check.variable], check.value)
    """

    def __init__(
        self,
        variable: str,
        value: ComparisonV,
        operator: typing.Callable[[ComparisonV, ComparisonV], bool],
    ):
        """Initialise variables.

        Args:
            variable: the variable name to use
            value: the value to use on the RHS of the expression
            operator: a logical operator to enable comparisons
        """
        self.variable = variable
        self.value = value
        self.operator = operator


##############################################################################
# BlackBoxes
##############################################################################


class BlackBoxLevel(enum.IntEnum):
    """
    A hint used for visualisation.

    Whether a behaviour is a blackbox entity that may be considered collapsible
    (i.e. everything in its subtree will not be visualised) by
    visualisation tools.
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
    Flag used by visualisation tools to configure visibility..

    Closely associated with the :py:class:`~py_trees.common.BlackBoxLevel` for a
    behaviour.

    This sets the visibility level to be used for visualisations.
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


def string_to_visibility_level(level: str) -> VisibilityLevel:
    """Will convert a string to a visibility level.

    Note that it will quietly return ALL if
    the string is not matched to any visibility level string identifier.

    Args:
        level: visibility level as a string

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

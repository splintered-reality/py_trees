#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/splintered-reality/py_trees/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
Library of common methods for the tests.

Oh my spaghettified magnificence,
Bless my noggin with a tickle from your noodly appendages!
"""

##############################################################################
# Imports
##############################################################################

import typing

from . import behaviour, blackboard, console, display, trees, visitors

##############################################################################
# Methods
##############################################################################


def print_assert_banner() -> None:
    """Print an assertion banner on stdout to indicate asserts will ensue."""
    print(console.green + "\n--------- Assertions ---------\n" + console.reset)


AssertResultType = typing.TypeVar("AssertResultType")


def print_assert_details(text: str, expected: AssertResultType, result: AssertResultType) -> None:
    """
    Pretty print the expected and actual results for an assertion.

    Args:
        text: human readable info about the assertion
        expected: expected result
        result: actual result
    """
    print(
        console.green
        + text
        + "." * (70 - len(text))
        + console.cyan
        + f"{expected}"
        + console.yellow
        + f" [{result}]"
        + console.reset
    )


def pre_tick_visitor(behaviour_tree: trees.BehaviourTree) -> None:
    """
    Tree tick banner.

    Args:
        behavior_tree: unused
    """
    print(f"\n--------- Run {behaviour_tree.count} ---------\n")


def tick_tree(
    root: behaviour.Behaviour,
    from_tick: int,
    to_tick: int,
    *,
    visitors: list[visitors.VisitorBase] | None = None,
    print_snapshot: bool = False,
    print_blackboard: bool = False,
) -> None:
    """
    Tick the tree for a specified # ticks and run a variety of debugging helpers.

    Args:
        root: the root of the tree to tick from
        from_tick: needed only to help provide accurate stdout information
        to_tick: with from_tick, used to determine the # ticks required
        visitors: a list of visitors to run on each tree tick
        print_snapshot: print ascii/unicode snapshots after each tick
        print_blackboard: display the blackboard and its update after each tick
    """
    if visitors is None:
        visitors = []
    print(f"\n================== Iteration {from_tick}-{to_tick} ==================\n")
    for i in range(from_tick, to_tick + 1):
        for visitor in visitors:
            visitor.initialise()
        print(f"\n--------- Run {i} ---------\n")
        for node in root.tick():
            for visitor in visitors:
                node.visit(visitor)
    if print_snapshot:
        print(console.green + "\nTree Snapshot" + console.reset)
        print(display.unicode_tree(root=root, show_status=True))
    if print_blackboard:
        print(display.unicode_blackboard())


def clear_blackboard() -> None:
    """Clear the blackboard, useful between tests."""
    blackboard.Blackboard.storage = {}
    blackboard.Blackboard.clients = {}
    blackboard.Blackboard.metadata = {}


def print_summary(nodes: list[behaviour.Behaviour]) -> None:
    """Print status details for a list of behaviours.

    Args:
        nodes: a list of behaviours to print information about.
    """
    print("\n--------- Summary ---------\n")
    for node in nodes:
        print(f"{node}")

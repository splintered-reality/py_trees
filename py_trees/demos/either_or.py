#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/splintered-reality/py_trees/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
A demonstration of the 'either_or' idiom.

.. argparse::
   :module: py_trees.demos.either_or
   :func: command_line_argument_parser
   :prog: py-trees-demo-either-or

.. graphviz:: dot/demo-either-or.dot

.. image:: images/either_or.gif
"""

##############################################################################
# Imports
##############################################################################

import argparse
import functools
import operator
import sys
import time
import typing

import py_trees
import py_trees.console as console

##############################################################################
# Classes
##############################################################################


def description(root: py_trees.behaviour.Behaviour) -> str:
    """
    Print description and usage information about the program.

    Returns:
       the program description string
    """
    content = "A demonstration of the 'either_or' idiom.\n\n"
    content += "This behaviour tree pattern enables triggering of subtrees\n"
    content += "with equal priority (first in, first served).\n"
    content += "\n"
    content += "EVENTS\n"
    content += "\n"
    content += " -  3 : joystick one enabled, task one starts\n"
    content += " -  5 : task one finishes\n"
    content += " -  6 : joystick two enabled, task two starts\n"
    content += " -  7 : joystick one enabled, task one ignored, task two continues\n"
    content += " -  8 : task two finishes\n"
    content += "\n"
    if py_trees.console.has_colours:
        banner_line = console.green + "*" * 79 + "\n" + console.reset
        s = banner_line
        s += console.bold_white + "Either Or".center(79) + "\n" + console.reset
        s += banner_line
        s += "\n"
        s += content
        s += "\n"
        s += banner_line
    else:
        s = content
    return s


def epilog() -> typing.Optional[str]:
    """
    Print a noodly epilog for --help.

    Returns:
       the noodly message
    """
    if py_trees.console.has_colours:
        return (
            console.cyan
            + "And his noodly appendage reached forth to tickle the blessed...\n"
            + console.reset
        )
    else:
        return None


def command_line_argument_parser() -> argparse.ArgumentParser:
    """
    Process command line arguments.

    Returns:
        the argument parser
    """
    parser = argparse.ArgumentParser(
        description=description(create_root()),
        epilog=epilog(),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    group = parser.add_mutually_exclusive_group()
    group.add_argument(
        "-r", "--render", action="store_true", help="render dot tree to file"
    )
    group.add_argument(
        "-i",
        "--interactive",
        action="store_true",
        help="pause and wait for keypress at each tick",
    )
    return parser


def pre_tick_handler(behaviour_tree: py_trees.trees.BehaviourTree) -> None:
    """Print a banner with current tick count prior to ticking the tree.

    Args:
       behaviour_tree: the tree to tick (used to fetch the count number)
    """
    print("\n--------- Run %s ---------\n" % behaviour_tree.count)


def post_tick_handler(
    snapshot_visitor: py_trees.visitors.SnapshotVisitor,
    behaviour_tree: py_trees.trees.BehaviourTree,
) -> None:
    """
    Print data about the part of the tree visited.

    Args:
        snapshot_handler: gather data about the part of the tree visited
        behaviour_tree: tree to gather data from
    """
    print(
        "\n"
        + py_trees.display.unicode_tree(
            root=behaviour_tree.root,
            visited=snapshot_visitor.visited,
            previously_visited=snapshot_visitor.previously_visited,
        )
    )
    print(py_trees.display.unicode_blackboard())


def create_root() -> py_trees.behaviour.Behaviour:
    """
    Create the root behaviour and it's subtree.

    Returns:
        the root behaviour
    """
    trigger_one = py_trees.decorators.FailureIsRunning(
        name="FisR", child=py_trees.behaviours.SuccessEveryN(name="Joystick 1", n=4)
    )
    trigger_two = py_trees.decorators.FailureIsRunning(
        name="FisR", child=py_trees.behaviours.SuccessEveryN(name="Joystick 2", n=7)
    )
    enable_joystick_one = py_trees.behaviours.SetBlackboardVariable(
        name="Joy1 - Enabled",
        variable_name="joystick_one",
        variable_value="enabled",
        overwrite=True,
    )
    enable_joystick_two = py_trees.behaviours.SetBlackboardVariable(
        name="Joy2 - Enabled",
        variable_name="joystick_two",
        variable_value="enabled",
        overwrite=True,
    )
    reset_joystick_one = py_trees.behaviours.SetBlackboardVariable(
        name="Joy1 - Disabled",
        variable_name="joystick_one",
        variable_value="disabled",
        overwrite=True,
    )
    reset_joystick_two = py_trees.behaviours.SetBlackboardVariable(
        name="Joy2 - Disabled",
        variable_name="joystick_two",
        variable_value="disabled",
        overwrite=True,
    )
    task_one = py_trees.behaviours.TickCounter(
        name="Task 1", duration=2, completion_status=py_trees.common.Status.SUCCESS
    )
    task_two = py_trees.behaviours.TickCounter(
        name="Task 2", duration=2, completion_status=py_trees.common.Status.SUCCESS
    )
    idle = py_trees.behaviours.Running(name="Idle")
    either_or = py_trees.idioms.either_or(
        name="Either Or",
        conditions=[
            py_trees.common.ComparisonExpression(
                "joystick_one", "enabled", operator.eq
            ),
            py_trees.common.ComparisonExpression(
                "joystick_two", "enabled", operator.eq
            ),
        ],
        subtrees=[task_one, task_two],
        namespace="either_or",
    )
    root = py_trees.composites.Parallel(
        name="Root",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=False),
    )
    reset = py_trees.composites.Sequence(name="Reset", memory=True)
    reset.add_children([reset_joystick_one, reset_joystick_two])
    joystick_one_events = py_trees.composites.Sequence(name="Joy1 Events", memory=True)
    joystick_one_events.add_children([trigger_one, enable_joystick_one])
    joystick_two_events = py_trees.composites.Sequence(name="Joy2 Events", memory=True)
    joystick_two_events.add_children([trigger_two, enable_joystick_two])
    tasks = py_trees.composites.Selector(name="Tasks", memory=False)
    tasks.add_children([either_or, idle])
    root.add_children([reset, joystick_one_events, joystick_two_events, tasks])
    return root


##############################################################################
# Main
##############################################################################


def main() -> None:
    """Entry point for the demo script."""
    args = command_line_argument_parser().parse_args()
    # py_trees.logging.level = py_trees.logging.Level.DEBUG
    root = create_root()
    print(description(root))

    ####################
    # Rendering
    ####################
    if args.render:
        py_trees.display.render_dot_tree(root)
        sys.exit()

    ####################
    # Tree Stewardship
    ####################
    behaviour_tree = py_trees.trees.BehaviourTree(root)
    behaviour_tree.add_pre_tick_handler(pre_tick_handler)
    behaviour_tree.visitors.append(py_trees.visitors.DebugVisitor())
    snapshot_visitor = py_trees.visitors.SnapshotVisitor()
    behaviour_tree.add_post_tick_handler(
        functools.partial(post_tick_handler, snapshot_visitor)
    )
    behaviour_tree.visitors.append(snapshot_visitor)
    behaviour_tree.setup(timeout=15)

    ####################
    # Tick Tock
    ####################
    if args.interactive:
        py_trees.console.read_single_keypress()
    for _unused_i in range(1, 11):
        try:
            behaviour_tree.tick()
            if args.interactive:
                py_trees.console.read_single_keypress()
            else:
                time.sleep(0.5)
        except KeyboardInterrupt:
            break
    print("\n")

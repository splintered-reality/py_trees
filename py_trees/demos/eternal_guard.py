#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/splintered-reality/py_trees/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
A demonstration of the 'eternal_guard' concept.

.. argparse::
   :module: py_trees.demos.eternal_guard
   :func: command_line_argument_parser
   :prog: py-trees-demo-eternal-guard

.. graphviz:: dot/demo-eternal-guard.dot

.. image:: images/eternal_guard.gif
"""

##############################################################################
# Imports
##############################################################################

import argparse
import functools
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
    content = "A demonstration of the 'eternal guard' concept.\n\n"
    content += "Two binary (F|S) conditional checks will fire every\n"
    content += "tick, thus providing a fail-fast mechanism for the\n"
    content += "long running sequence of tasks behind them.\n"
    content += "\n"
    if py_trees.console.has_colours:
        banner_line = console.green + "*" * 79 + "\n" + console.reset
        s = banner_line
        s += console.bold_white + "Eternal Guard".center(79) + "\n" + console.reset
        s += banner_line
        s += "\n"
        s += content
        s += "\n"
        s += py_trees.display.unicode_tree(root)
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
    eternal_guard = py_trees.composites.Sequence(name="Eternal Guard", memory=False)
    condition_one = py_trees.behaviours.StatusQueue(
        name="Condition 1",
        queue=[
            py_trees.common.Status.SUCCESS,
            py_trees.common.Status.FAILURE,
            py_trees.common.Status.SUCCESS,
        ],
        eventually=py_trees.common.Status.SUCCESS,
    )
    condition_two = py_trees.behaviours.StatusQueue(
        name="Condition 2",
        queue=[
            py_trees.common.Status.SUCCESS,
            py_trees.common.Status.SUCCESS,
            py_trees.common.Status.FAILURE,
        ],
        eventually=py_trees.common.Status.SUCCESS,
    )
    task_sequence = py_trees.composites.Sequence(name="Task Sequence", memory=True)
    task_one = py_trees.behaviours.Success(name="Worker 1")
    task_two = py_trees.behaviours.Running(name="Worker 2")

    eternal_guard.add_children([condition_one, condition_two, task_sequence])
    task_sequence.add_children([task_one, task_two])
    return eternal_guard


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

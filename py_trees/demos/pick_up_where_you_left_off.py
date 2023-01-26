#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/splintered-reality/py_trees/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
Demonstrates the 'pick up where you left off idiom'.

.. argparse::
   :module: py_trees.demos.pick_up_where_you_left_off
   :func: command_line_argument_parser
   :prog: py-trees-demo-pick-up-where-you-left-off

.. graphviz:: dot/pick_up_where_you_left_off.dot

.. image:: images/pick_up_where_you_left_off.gif
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
    content = "A demonstration of the 'pick up where you left off' idiom.\n\n"
    content += "A common behaviour tree pattern that allows you to resume\n"
    content += "work after being interrupted by a high priority interrupt.\n"
    content += "\n"
    content += "EVENTS\n"
    content += "\n"
    content += " -  2 : task one done, task two running\n"
    content += " -  3 : high priority interrupt\n"
    content += " -  7 : task two restarts\n"
    content += " -  9 : task two done\n"
    content += "\n"
    if py_trees.console.has_colours:
        banner_line = console.green + "*" * 79 + "\n" + console.reset
        s = banner_line
        s += (
            console.bold_white
            + "Pick Up Where you Left Off".center(79)
            + "\n"
            + console.reset
        )
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
    """Print a banner immediately before every tick of the tree.

    Args:
        behaviour_tree (:class:`~py_trees.trees.BehaviourTree`): the tree custodian

    """
    print("\n--------- Run %s ---------\n" % behaviour_tree.count)


def post_tick_handler(
    snapshot_visitor: py_trees.visitors.SnapshotVisitor,
    behaviour_tree: py_trees.trees.BehaviourTree,
) -> None:
    """Print an ascii tree with the current snapshot status."""
    print(
        "\n"
        + py_trees.display.unicode_tree(
            root=behaviour_tree.root,
            visited=snapshot_visitor.visited,
            previously_visited=snapshot_visitor.previously_visited,
        )
    )


def create_root() -> py_trees.behaviour.Behaviour:
    """
    Create the root behaviour and it's subtree.

    Returns:
        the root behaviour
    """
    task_one = py_trees.behaviours.StatusQueue(
        name="Task 1",
        queue=[
            py_trees.common.Status.RUNNING,
            py_trees.common.Status.RUNNING,
        ],
        eventually=py_trees.common.Status.SUCCESS,
    )
    task_two = py_trees.behaviours.StatusQueue(
        name="Task 2",
        queue=[
            py_trees.common.Status.RUNNING,
            py_trees.common.Status.RUNNING,
        ],
        eventually=py_trees.common.Status.SUCCESS,
    )
    high_priority_interrupt = py_trees.decorators.RunningIsFailure(
        name="Running is Failure",
        child=py_trees.behaviours.Periodic(name="High Priority", n=3),
    )
    piwylo = py_trees.idioms.pick_up_where_you_left_off(
        name="Pick Up\nWhere You\nLeft Off", tasks=[task_one, task_two]
    )
    root = py_trees.composites.Selector(name="Root", memory=False)
    root.add_children([high_priority_interrupt, piwylo])

    return root


##############################################################################
# Main
##############################################################################


def main() -> None:
    """Entry point for the demo script."""
    args = command_line_argument_parser().parse_args()
    py_trees.logging.level = py_trees.logging.Level.DEBUG
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

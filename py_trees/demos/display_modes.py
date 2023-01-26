#!/usr/bin/env python3
#
# License: BSD
#   https://raw.githubusercontent.com/splintered-reality/py_trees/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
Demonstrates usage of the ascii/unicode display modes.

.. argparse::
   :module: py_trees.demos.display_modes
   :func: command_line_argument_parser
   :prog: py-trees-demo-display-modes

.. figure:: images/display_modes.png
   :align: center

   Console Screenshot
"""

##############################################################################
# Imports
##############################################################################

import argparse
import itertools
import typing

import py_trees
import py_trees.console as console

##############################################################################
# Classes
##############################################################################


def description() -> str:
    """
    Print description and usage information about the program.

    Returns:
       the program description string
    """
    content = "Demonstrates usage of the ascii/unicode display modes.\n"
    content += "\n"
    content += "...\n"
    content += "...\n"

    if py_trees.console.has_colours:
        banner_line = console.green + "*" * 79 + "\n" + console.reset
        s = banner_line
        s += console.bold_white + "Display Modes".center(79) + "\n" + console.reset
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
        description=description(),
        epilog=epilog(),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    return parser


def create_root() -> py_trees.behaviour.Behaviour:
    """
    Create the tree to be ticked/displayed.

    Returns:
        the root of the tree
    """
    root = py_trees.composites.Sequence(name="root", memory=True)
    child = py_trees.composites.Sequence(name="child1", memory=True)
    child2 = py_trees.composites.Sequence(name="child2", memory=True)
    child3 = py_trees.composites.Sequence(name="child3", memory=True)
    root.add_child(child)
    root.add_child(child2)
    root.add_child(child3)
    queue = [py_trees.common.Status.RUNNING]
    eventually = py_trees.common.Status.SUCCESS
    child.add_child(
        py_trees.behaviours.StatusQueue(name="RS", queue=queue, eventually=eventually)
    )
    child2.add_child(
        py_trees.behaviours.StatusQueue(name="RS", queue=queue, eventually=eventually)
    )
    child2_child1 = py_trees.composites.Sequence(name="Child2_child1", memory=True)
    child2_child1.add_child(
        py_trees.behaviours.StatusQueue(name="RS", queue=queue, eventually=eventually)
    )
    child2.add_child(child2_child1)
    child3.add_child(
        py_trees.behaviours.StatusQueue(name="RS", queue=queue, eventually=eventually)
    )
    return root


##############################################################################
# Main
##############################################################################


def main() -> None:
    """Entry point for the demo script."""
    _ = (
        command_line_argument_parser().parse_args()
    )  # configuration only, no args to process
    print(description())
    print(
        "-------------------------------------------------------------------------------"
    )
    print("$ py_trees.blackboard.Client(name='Blackboard')")
    print("$ foo.register_key(key='dude', access=py_trees.common.Access.WRITE)")
    print("$ foo.register_key(key='/dudette', access=py_trees.common.Access.WRITE)")
    print("$ foo.register_key(key='/foo/bar/wow', access=py_trees.common.Access.WRITE)")
    print(
        "-------------------------------------------------------------------------------"
    )

    snapshot_visitor = py_trees.visitors.SnapshotVisitor()
    tree = py_trees.trees.BehaviourTree(create_root())
    tree.add_visitor(snapshot_visitor)

    for tick in range(2):
        tree.tick()
        for show_visited, show_status in itertools.product(
            [False, True], [False, True]
        ):
            console.banner(
                "Tick {} / show_only_visited=={} / show_status=={}".format(
                    tick, show_visited, show_status
                )
            )
            print(
                py_trees.display.unicode_tree(
                    tree.root,
                    show_status=show_status,
                    show_only_visited=show_visited,
                    visited=snapshot_visitor.visited,
                    previously_visited=snapshot_visitor.previously_visited,
                )
            )
            print()

#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/splintered-reality/py_trees/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
Demonstrates sequences in action.

.. argparse::
   :module: py_trees.demos.sequence
   :func: command_line_argument_parser
   :prog: py-trees-demo-sequence

.. graphviz:: dot/demo-sequence.dot

.. image:: images/sequence.gif
"""

##############################################################################
# Imports
##############################################################################

import argparse
import sys
import time
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
    content = "Demonstrates sequences in action.\n\n"
    content += (
        "A sequence is populated with 2-tick jobs that are allowed to run through to\n"
    )
    content += "completion.\n"

    if py_trees.console.has_colours:
        banner_line = console.green + "*" * 79 + "\n" + console.reset
        s = banner_line
        s += console.bold_white + "Sequences".center(79) + "\n" + console.reset
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
    parser.add_argument(
        "-r", "--render", action="store_true", help="render dot tree to file"
    )
    return parser


def create_root() -> py_trees.behaviour.Behaviour:
    """
    Create the root behaviour and it's subtree.

    Returns:
        the root behaviour
    """
    root = py_trees.composites.Sequence(name="Sequence", memory=True)
    for action in ["Action 1", "Action 2", "Action 3"]:
        rssss = py_trees.behaviours.StatusQueue(
            name=action,
            queue=[
                py_trees.common.Status.RUNNING,
                py_trees.common.Status.SUCCESS,
            ],
            eventually=py_trees.common.Status.SUCCESS,
        )
        root.add_child(rssss)
    return root


##############################################################################
# Main
##############################################################################


def main() -> None:
    """Entry point for the demo script."""
    args = command_line_argument_parser().parse_args()
    print(description())
    py_trees.logging.level = py_trees.logging.Level.DEBUG

    root = create_root()

    ####################
    # Rendering
    ####################
    if args.render:
        py_trees.display.render_dot_tree(root)
        sys.exit()

    ####################
    # Execute
    ####################
    root.setup_with_descendants()
    for i in range(1, 6):
        try:
            print("\n--------- Tick {0} ---------\n".format(i))
            root.tick_once()
            print("\n")
            print(py_trees.display.unicode_tree(root=root, show_status=True))
            time.sleep(1.0)
        except KeyboardInterrupt:
            break
    print("\n")

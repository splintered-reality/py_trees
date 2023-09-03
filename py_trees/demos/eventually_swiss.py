#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/splintered-reality/py_trees/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
Demonstrate a swiss-knife variant of the eventually idiom.

.. argparse::
   :module: py_trees.demos.eventually_swiss
   :func: command_line_argument_parser
   :prog: py-trees-demo-eventually-swiss-program

.. graphviz:: dot/demo-eventually-swiss.dot

.. image:: images/demo-eventually-swiss.png

"""

##############################################################################
# Imports
##############################################################################

import argparse
import sys
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
    content = "The swiss-knife version of the 'eventually' idiom.\n\n"
    content += "A counter is started and on successful completion\n"
    content += "(or otherwise), the result is recorded on the blackboard.\n"
    content += "\n"
    content += "NB: The demo is run twice - on the first, the count\n"
    content += "terminates with SUCCESS and on the second, with FAILURE.\n"
    content += "\n"
    content += "EVENTS\n"
    content += " -  1  : counter is started\n"
    content += " -  2a : count completes with SUCCESS||FAILURE\n"
    content += " -  2b : eventually pathways are triggered\n"
    content += " -  2c : blackboard count variable is set to true||false\n"
    content += "\n"
    if py_trees.console.has_colours:
        banner_line = console.green + "*" * 79 + "\n" + console.reset
        s = banner_line
        s += (
            console.bold_white
            + "Eventually - Swiss Variant".center(79)
            + "\n"
            + console.reset
        )
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
        description=description(create_root(py_trees.common.Status.SUCCESS)),
        epilog=epilog(),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    group = parser.add_mutually_exclusive_group()
    group.add_argument(
        "-r", "--render", action="store_true", help="render dot tree to file"
    )
    return parser


def create_root(
    expected_work_termination_result: py_trees.common.Status,
) -> py_trees.behaviour.Behaviour:
    """
    Create the root behaviour and it's subtree.

    Returns:
        the root behaviour
    """
    set_result_true = py_trees.behaviours.SetBlackboardVariable(
        name="SetResultTrue",
        variable_name="result",
        variable_value=True,
        overwrite=True,
    )
    set_result_false = py_trees.behaviours.SetBlackboardVariable(
        name="SetResultFalse",
        variable_name="result",
        variable_value=False,
        overwrite=True,
    )
    worker = py_trees.behaviours.TickCounter(
        name="Counter", duration=1, completion_status=expected_work_termination_result
    )
    root = py_trees.idioms.eventually_swiss(
        name="Count with Result",
        workers=[worker],
        on_failure=set_result_false,
        on_success=set_result_true,
    )
    return root


##############################################################################
# Main
##############################################################################


def main() -> None:
    """Entry point for the demo script."""
    args = command_line_argument_parser().parse_args()
    # py_trees.logging.level = py_trees.logging.Level.DEBUG
    print(description(create_root(py_trees.common.Status.SUCCESS)))

    ####################
    # Rendering
    ####################
    if args.render:
        py_trees.display.render_dot_tree(create_root(py_trees.common.Status.SUCCESS))
        sys.exit()

    for status in (py_trees.common.Status.SUCCESS, py_trees.common.Status.FAILURE):
        py_trees.blackboard.Blackboard.clear()
        console.banner(f"Experiment - Terminate with {status}")
        root = create_root(status)
        root.tick_once()
        print(py_trees.display.unicode_tree(root=root, show_status=True))
        print(py_trees.display.unicode_blackboard())
        root.tick_once()
        print(py_trees.display.unicode_tree(root=root, show_status=True))
        print(py_trees.display.unicode_blackboard())

    print("\n")

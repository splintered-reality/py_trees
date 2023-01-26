#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/splintered-reality/py_trees/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
Demonstrate the context switching design pattern.

.. argparse::
   :module: py_trees.demos.context_switching
   :func: command_line_argument_parser
   :prog: py-trees-demo-context-switching

.. graphviz:: dot/demo-context_switching.dot

.. image:: images/context_switching.gif
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
    content = "Demonstrates context switching with parallels and sequences.\n"
    content += "\n"
    content += (
        "A context switching behaviour is run in parallel with a work sequence.\n"
    )
    content += (
        "Switching the context occurs in the initialise() and terminate() methods\n"
    )
    content += (
        "of the context switching behaviour. Note that whether the sequence results\n"
    )
    content += (
        "in failure or success, the context switch behaviour will always call the\n"
    )
    content += (
        "terminate() method to restore the context. It will also call terminate()\n"
    )
    content += (
        "to restore the context in the event of a higher priority parent cancelling\n"
    )
    content += "this parallel subtree.\n"
    if py_trees.console.has_colours:
        banner_line = console.green + "*" * 79 + "\n" + console.reset
        s = banner_line
        s += console.bold_white + "Context Switching".center(79) + "\n" + console.reset
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


class ContextSwitch(py_trees.behaviour.Behaviour):
    """
    An example of a context switching class.

    This class sets (in ``initialise()``)
    and restores a context (in ``terminate()``). Use in parallel with a
    sequence/subtree that does the work while in this context.

    .. attention:: Simply setting a pair of behaviours (set and reset context) on
        either end of a sequence will not suffice for context switching. In the case
        that one of the work behaviours in the sequence fails, the final reset context
        switch will never trigger.
    """

    def __init__(self, name: str = "ContextSwitch"):
        """Initialise with a behaviour name."""
        super(ContextSwitch, self).__init__(name)
        self.feedback_message = "no context"

    def initialise(self) -> None:
        """Backup and set a new context."""
        self.logger.debug("%s.initialise()[switch context]" % (self.__class__.__name__))
        # Some actions that:
        #   1. retrieve the current context from somewhere
        #   2. cache the context internally
        #   3. apply a new context
        self.feedback_message = "new context"

    def update(self) -> py_trees.common.Status:
        """Just returns RUNNING while it waits for other activities to finish."""
        self.logger.debug(
            "%s.update()[RUNNING][%s]"
            % (self.__class__.__name__, self.feedback_message)
        )
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status: py_trees.common.Status) -> None:
        """Restore the context with the previously backed up context."""
        self.logger.debug(
            "%s.terminate()[%s->%s][restore context]"
            % (self.__class__.__name__, self.status, new_status)
        )
        # Some actions that:
        #   1. restore the cached context
        self.feedback_message = "restored context"


def create_root() -> py_trees.behaviour.Behaviour:
    """
    Create the root behaviour and it's subtree.

    Returns:
        the root behaviour
    """
    root = py_trees.composites.Parallel(
        name="Parallel", policy=py_trees.common.ParallelPolicy.SuccessOnOne()
    )
    context_switch = ContextSwitch(name="Context")
    sequence = py_trees.composites.Sequence(name="Sequence", memory=True)
    for job in ["Action 1", "Action 2"]:
        success_after_two = py_trees.behaviours.StatusQueue(
            name=job,
            queue=[py_trees.common.Status.RUNNING, py_trees.common.Status.RUNNING],
            eventually=py_trees.common.Status.SUCCESS,
        )
        sequence.add_child(success_after_two)
    root.add_child(context_switch)
    root.add_child(sequence)
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
            print("{}".format(py_trees.display.unicode_tree(root, show_status=True)))
            time.sleep(1.0)
        except KeyboardInterrupt:
            break
    print("\n")

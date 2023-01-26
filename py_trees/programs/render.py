#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/splintered-reality/py_trees/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
Program to render dot/svg/png graphs of methods that return a pytree.

.. argparse::
   :module: py_trees.programs.render
   :func: command_line_argument_parser
   :prog: py-trees-render

.. image:: images/render.gif
"""

##############################################################################
# Imports
##############################################################################

import argparse
import importlib
import json
import sys
import typing

import py_trees
import py_trees.console as console

##############################################################################
# Classes
##############################################################################


def examples() -> typing.List[str]:
    """
    Usage examples as a string message for --help's description.

    Return:
       the string message
    """
    prefix = console.cyan + "py-trees-render" + console.yellow
    examples = [
        prefix + " py_trees.demos.stewardship.create_tree" + console.reset,
        prefix + " --with-blackboard-variables" + console.reset,
        prefix + " --name=foo py_trees.demos.stewardship.create_tree" + console.reset,
        prefix
        + ' --kwargs=\'{"level":"all"}\' py_trees.demos.dot_graphs.create_tree'
        + console.reset,
    ]
    return examples


def description() -> str:
    """
    Print description and usage information about the program.

    Returns:
       the program description string
    """
    short = "Point this program at a method which creates a root to render to dot/svg/png.\n\n"

    if py_trees.console.has_colours:
        banner_line = console.green + "*" * 79 + "\n" + console.reset
        s = banner_line
        s += console.bold_white + "Trees".center(79) + "\n" + console.reset
        s += banner_line
        s += "\n"
        s += short
        s += "\n"
        s += console.bold + "Examples" + console.reset + "\n\n"
        s += "\n".join(["    $ " + example for example in examples()])
        s += "\n\n"
        s += banner_line
    else:
        # for sphinx documentation (doesn't like raw text)
        s = short
        s += "\n"
        s += console.bold + "**Examples**" + console.reset + "\n\n"
        s += ".. code-block:: bash\n"
        s += "    \n"
        s += "\n".join(["    $ {0}".format(example) for example in examples()])
        s += "\n"
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
        "method",
        default=None,
        help="space separated list of blackboard variables to watch",
    )
    parser.add_argument(
        "-l",
        "--level",
        action="store",
        default="fine_detail",
        choices=["all", "fine_detail", "detail", "component", "big_picture"],
        help="visibility level",
    )
    parser.add_argument(
        "-n",
        "--name",
        default=None,
        help="name to use for the created files (defaults to the root behaviour name)",
    )
    parser.add_argument(
        "-k",
        "--kwargs",
        default="{}",
        type=json.loads,
        help="dictionary of keyword arguments to the method",
    )
    parser.add_argument(
        "-b",
        "--with-blackboard-variables",
        default=False,
        action="store_true",
        help="add nodes for the blackboard variables",
    )
    parser.add_argument(
        "-v",
        "--verbose",
        default=False,
        action="store_true",
        help="embellish each node in the dot graph with extra information",
    )
    return parser


##############################################################################
# Main
##############################################################################


def main() -> None:
    """Entry point."""
    args = command_line_argument_parser().parse_args()
    args.enum_level = py_trees.common.string_to_visibility_level(args.level)
    (module_or_class_name, method_name) = args.method.rsplit(".", 1)
    class_name = None

    try:
        module_itself = importlib.import_module(module_or_class_name)
        module_name = module_or_class_name
    except ImportError:
        # maybe it's a class?
        (module_name, class_name) = module_or_class_name.rsplit(".", 1)
        try:
            module_itself = importlib.import_module(module_name)
        except ImportError:
            console.logerror(
                "Could not import module [{0}]".format(module_or_class_name)
            )
            sys.exit(1)
    if class_name is not None:
        class_type = getattr(module_itself, class_name)
        # first guess - it's a static method
        method_itself = getattr(class_type, method_name)
        try:
            root = method_itself(**(args.kwargs))
        except TypeError:  # oops, it's an instance method
            try:
                method_itself = getattr(class_type(), method_name)
            except TypeError:
                console.logerror(
                    "Can only instantiate class methods if the class __init__ has no non-default arguments"
                )
                sys.exit(1)
            root = method_itself(**(args.kwargs))
    else:
        method_itself = getattr(module_itself, method_name)
        root = method_itself(**(args.kwargs))

    # TODO figure out how to insert keyword arguments
    py_trees.display.render_dot_tree(
        root,
        args.enum_level,
        args.name,
        with_blackboard_variables=args.with_blackboard_variables,
        with_qualified_names=args.verbose,
    )

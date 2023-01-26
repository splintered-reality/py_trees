#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/splintered-reality/py_trees/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
Demonstrates usage of blackboard namespaces.

.. argparse::
   :module: py_trees.demos.blackboard_namespaces
   :func: command_line_argument_parser
   :prog: py-trees-demo-blackboard-namespaces

.. figure:: images/blackboard_namespaces.png
   :align: center

   Console Screenshot
"""

##############################################################################
# Imports
##############################################################################

import argparse
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
    content = "Demonstrates usage of blackboard namespaces.\n"
    content += "\n"

    if py_trees.console.has_colours:
        banner_line = console.green + "*" * 79 + "\n" + console.reset
        s = banner_line
        s += console.bold_white + "Blackboard".center(79) + "\n" + console.reset
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
    blackboard = py_trees.blackboard.Client(name="Blackboard")
    blackboard.register_key(key="dude", access=py_trees.common.Access.WRITE)
    blackboard.register_key(key="/dudette", access=py_trees.common.Access.WRITE)
    blackboard.register_key(key="/foo/bar/wow", access=py_trees.common.Access.WRITE)
    print(blackboard)
    print(
        "-------------------------------------------------------------------------------"
    )
    print("$ blackboard.dude = 'Bob'")
    print("$ blackboard.dudette = 'Jade'")
    print(
        "-------------------------------------------------------------------------------"
    )
    blackboard.dude = "Bob"
    blackboard.dudette = "Jade"
    print(py_trees.display.unicode_blackboard())
    print(
        "-------------------------------------------------------------------------------"
    )
    print("$ blackboard.foo.bar.wow = 'foobar'")
    print(
        "-------------------------------------------------------------------------------"
    )
    blackboard.foo.bar.wow = "foobar"
    print(py_trees.display.unicode_blackboard())
    print(
        "-------------------------------------------------------------------------------"
    )
    print("$ py_trees.blackboard.Client(name='Foo', namespace='foo')")
    print("$ foo.register_key(key='awesome', access=py_trees.common.Access.WRITE)")
    print("$ foo.register_key(key='/brilliant', access=py_trees.common.Access.WRITE)")
    print("$ foo.register_key(key='/foo/clever', access=py_trees.common.Access.WRITE)")
    print(
        "-------------------------------------------------------------------------------"
    )
    foo = py_trees.blackboard.Client(name="Foo", namespace="foo")
    foo.register_key(key="awesome", access=py_trees.common.Access.WRITE)
    # TODO: should /brilliant be namespaced or go directly to root?
    foo.register_key(key="/brilliant", access=py_trees.common.Access.WRITE)
    # absolute names are ok, so long as they include the namespace
    foo.register_key(key="/foo/clever", access=py_trees.common.Access.WRITE)
    print(foo)
    print(
        "-------------------------------------------------------------------------------"
    )
    print("$ foo.awesome = True")
    print("$ foo.set('/brilliant', False)")
    print("$ foo.clever = True")
    print(
        "-------------------------------------------------------------------------------"
    )
    foo.awesome = True
    # Only accessable via set since it's not in the namespace
    foo.set("/brilliant", False)
    # This will fail since it looks for the namespaced /foo/brilliant key
    # foo.brilliant = False
    foo.clever = True
    print(py_trees.display.unicode_blackboard())

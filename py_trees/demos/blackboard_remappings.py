#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/splintered-reality/py_trees/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
.. argparse::
   :module: py_trees.demos.blackboard_remappings
   :func: command_line_argument_parser
   :prog: py-trees-demo-blackboard-remappings

.. figure:: images/blackboard_remappings.png
   :align: center

   Console Screenshot
"""

##############################################################################
# Imports
##############################################################################

import argparse
import py_trees
import typing

import py_trees.console as console

##############################################################################
# Classes
##############################################################################


def description():
    content = "Demonstrates usage of blackbord remappings.\n"
    content += "\n"
    content += "Demonstration is via an exemplar behaviour making use of remappings..\n"

    if py_trees.console.has_colours:
        banner_line = console.green + "*" * 79 + "\n" + console.reset
        s = "\n"
        s += banner_line
        s += console.bold_white + "Blackboard".center(79) + "\n" + console.reset
        s += banner_line
        s += "\n"
        s += content
        s += "\n"
        s += banner_line
    else:
        s = content
    return s


def epilog():
    if py_trees.console.has_colours:
        return console.cyan + "And his noodly appendage reached forth to tickle the blessed...\n" + console.reset
    else:
        return None


def command_line_argument_parser():
    parser = argparse.ArgumentParser(description=description(),
                                     epilog=epilog(),
                                     formatter_class=argparse.RawDescriptionHelpFormatter,
                                     )
    return parser


class Remap(py_trees.behaviour.Behaviour):
    """
    Custom writer that submits a more complicated variable to the blackboard.
    """
    def __init__(self, name: str, remap_to: typing.Dict[str, str]):
        super().__init__(name=name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key(
            key="/foo/bar/wow",
            access=py_trees.common.Access.WRITE,
            remap_to=remap_to["/foo/bar/wow"]
        )

    def update(self):
        """
        Write a dictionary to the blackboard and return :data:`~py_trees.common.Status.SUCCESS`.
        """
        self.logger.debug("%s.update()" % (self.__class__.__name__))
        self.blackboard.foo.bar.wow = "colander"

        return py_trees.common.Status.SUCCESS

##############################################################################
# Main
##############################################################################


def main():
    """
    Entry point for the demo script.
    """
    args = command_line_argument_parser().parse_args()
    print(description())
    py_trees.logging.level = py_trees.logging.Level.DEBUG
    py_trees.blackboard.Blackboard.enable_activity_stream(maximum_size=100)
    root = Remap(name="Remap", remap_to={"/foo/bar/wow": "/parameters/wow"})

    ####################
    # Execute
    ####################
    root.tick_once()
    print(root.blackboard)
    print(py_trees.display.unicode_blackboard())
    print(py_trees.display.unicode_blackboard_activity_stream())

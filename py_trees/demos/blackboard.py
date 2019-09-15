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
   :module: py_trees.demos.blackboard
   :func: command_line_argument_parser
   :prog: py-trees-demo-blackboard

.. graphviz:: dot/demo-blackboard.dot
   :align: center

.. image:: images/blackboard.gif
"""

##############################################################################
# Imports
##############################################################################

import argparse
import py_trees
import sys

import py_trees.console as console

##############################################################################
# Classes
##############################################################################


def description():
    content = "Demonstrates usage of the blackboard and related behaviours.\n"
    content += "\n"
    content += "A sequence is populated with a few behaviours that exercise\n"
    content += "reading and writing on the Blackboard in interesting ways.\n"

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
    parser.add_argument('-r', '--render', action='store_true', help='render dot tree to file')
    return parser


class BlackboardWriter(py_trees.behaviour.Behaviour):
    """
    Custom writer that submits a more complicated variable to the blackboard.
    """
    def __init__(self, name="Writer"):
        super(BlackboardWriter, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        self.blackboard = py_trees.blackboard.Blackboard(
            name=self.name,
            unique_identifier=self.id,
            write={"spaghetti"},
            read={"dude"}
        )

    def update(self):
        """
        Write a dictionary to the blackboard and return :data:`~py_trees.common.Status.SUCCESS`.
        """
        self.logger.debug("%s.update()" % (self.__class__.__name__))
        try:
            unused = self.blackboard.dude
        except KeyError:
            pass
        try:
            unused = self.blackboard.dudette
        except AttributeError:
            pass
        try:
            self.blackboard.dudette = "Jane"
        except AttributeError:
            pass
        self.blackboard.spaghetti = {"type": "Carbonara", "quantity": 1}
        self.blackboard.spaghetti = {"type": "Gnocchi", "quantity": 2}
        try:
            self.blackboard.set("spaghetti", {"type": "Bolognese", "quantity": 3}, overwrite=False)
        except AttributeError:
            pass
        return py_trees.common.Status.SUCCESS


def create_root():
    root = py_trees.composites.Sequence("Blackboard Demo")
    set_blackboard_variable = py_trees.blackboard.SetBlackboardVariable(name="Set Foo", variable_name="foo", variable_value="bar")
    write_blackboard_variable = BlackboardWriter(name="Writer")
    check_blackboard_variable = py_trees.blackboard.CheckBlackboardVariable(name="Check Foo", variable_name="foo", expected_value="bar")
    root.add_children([set_blackboard_variable, write_blackboard_variable, check_blackboard_variable])
    return root

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

    root = create_root()

    ####################
    # Rendering
    ####################
    if args.render:
        py_trees.display.render_dot_tree(root, with_blackboard_variables=True)
        sys.exit()

    ####################
    # Execute
    ####################
    root.setup_with_descendants()
    print("\n--------- Tick 0 ---------\n")
    root.tick_once()
    print("\n")
    print(py_trees.display.unicode_tree(root, show_status=True))
    print("--------------------------\n")
    print(py_trees.display.unicode_blackboard())
    print("--------------------------\n")
    print(py_trees.display.unicode_blackboard(display_only_key_metadata=True))
    print("--------------------------\n")
    blackboard = py_trees.blackboard.Blackboard(name="Unsetter", write={"foo"})
    blackboard.unset("foo")
    print(py_trees.display.unicode_blackboard_activity_stream())

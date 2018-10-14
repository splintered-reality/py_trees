#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/stonier/py_trees/devel/LICENSE
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
    content += "A sequence is populated with a default set blackboard variable\n"
    content += "behaviour, a custom write to blackboard behaviour that writes\n"
    content += "a more complicated structure, and finally a default check\n"
    content += "blackboard variable beheaviour that looks for the first variable.\n"

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
        self.blackboard = py_trees.blackboard.Blackboard()

    def update(self):
        """
        Write a dictionary to the blackboard and return :data:`~py_trees.common.Status.SUCCESS`.
        """
        self.logger.debug("%s.update()" % (self.__class__.__name__))
        self.blackboard.spaghetti = {"type": "Gnocchi", "quantity": 2}
        return py_trees.common.Status.SUCCESS


def create_tree():
    root = py_trees.composites.Sequence("Sequence")
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

    tree = create_tree()

    ####################
    # Rendering
    ####################
    if args.render:
        py_trees.display.render_dot_tree(tree)
        sys.exit()

    ####################
    # Execute
    ####################
    tree.setup(timeout=15)
    print("\n--------- Tick 0 ---------\n")
    tree.tick_once()
    print("\n")
    py_trees.display.print_ascii_tree(tree, show_status=True)
    print("\n")
    print(py_trees.blackboard.Blackboard())

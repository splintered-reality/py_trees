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
import py_trees
import sys
import time

import py_trees.console as console

##############################################################################
# Classes
##############################################################################


def description():
    content = "Demonstrates sequences in action.\n\n"
    content += "A sequence is populated with 2-tick jobs that are allowed to run through to\n"
    content += "completion.\n"

    if py_trees.console.has_colours:
        banner_line = console.green + "*" * 79 + "\n" + console.reset
        s = "\n"
        s += banner_line
        s += console.bold_white + "Sequences".center(79) + "\n" + console.reset
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


def create_tree():
    root = py_trees.composites.Sequence("Sequence")
    for action in ["Action 1", "Action 2", "Action 3"]:
        success_after_two = py_trees.behaviours.Count(name=action,
                                                      fail_until=0,
                                                      running_until=1,
                                                      success_until=10)
        root.add_child(success_after_two)
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
    for i in range(1, 6):
        try:
            print("\n--------- Tick {0} ---------\n".format(i))
            tree.tick_once()
            print("\n")
            py_trees.display.print_ascii_tree(tree, show_status=True)
            time.sleep(1.0)
        except KeyboardInterrupt:
            break
    print("\n")

#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/stonier/py_trees_suite/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
Code for the behaviour lifecycle demo program.
---
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
    content = "Demonstrates priority switching with selectors\n"
    if py_trees.console.has_colours:
        banner_line = console.green + "*" * 79 + "\n" + console.reset
        s = "\n"
        s += banner_line
        s += console.bold_white + "Selectors".center(79) + "\n" + console.reset
        s += banner_line
        s += "\n"
        s += content
        s += "\n"
        s += banner_line
    else:
        s = content
    return s


def command_line_argument_parser():
    parser = argparse.ArgumentParser(description=description(),
                                     epilog=console.cyan + "And his noodly appendage reached forth to tickle the blessed...\n" + console.reset,
                                     formatter_class=argparse.RawDescriptionHelpFormatter,
                                     )
    parser.add_argument('-r', '--render', action='store_true', help='render dot tree to file')
    return parser


def create_tree():
    root = py_trees.composites.Selector("Selector")
    success_after_two = py_trees.behaviours.Count(name="After Two",
                                                  fail_until=2,
                                                  running_until=2,
                                                  success_until=10)
    always_running = py_trees.behaviours.Running(name="Running")
    root.add_children([success_after_two, always_running])
    return root


##############################################################################
# Main
##############################################################################

def main():
    """
    Entry point for the demo behaviours lifecycle script.
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
    for i in range(1, 4):
        try:
            print("\n--------- Tick {0} ---------\n".format(i))
            tree.tick_once()
            print("\n")
            py_trees.display.print_ascii_tree(tree, show_status=True)
            time.sleep(1.0)
        except KeyboardInterrupt:
            break

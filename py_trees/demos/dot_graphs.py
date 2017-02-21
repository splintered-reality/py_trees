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
   :module: py_trees.demos.dot_graphs
   :func: command_line_argument_parser
   :prog: py-trees-demo-dot-graphs

.. graphviz:: dot/demo-dot-graphs.dot

"""

##############################################################################
# Imports
##############################################################################

import argparse
import subprocess
import py_trees

import py_trees.console as console

##############################################################################
# Classes
##############################################################################


def description():
    name = "py-trees-demo-dot-graphs"
    content = "Renders a dot graph for a simple tree, with blackboxes.\n"
    if py_trees.console.has_colours:
        banner_line = console.green + "*" * 79 + "\n" + console.reset
        s = "\n"
        s += banner_line
        s += console.bold_white + "Dot Graphs".center(79) + "\n" + console.reset
        s += banner_line
        s += "\n"
        s += content
        s += "\n"
        s += console.white
        s += console.bold + "    Generate Full Dot Graph" + console.reset + "\n"
        s += "\n"
        s += console.cyan + "        {0}".format(name) + console.reset + "\n"
        s += "\n"
        s += console.bold + "    With Varying Visibility Levels" + console.reset + "\n"
        s += "\n"
        s += console.cyan + "        {0}".format(name) + console.yellow + " --level=all" + console.reset + "\n"
        s += console.cyan + "        {0}".format(name) + console.yellow + " --level=detail" + console.reset + "\n"
        s += console.cyan + "        {0}".format(name) + console.yellow + " --level=component" + console.reset + "\n"
        s += console.cyan + "        {0}".format(name) + console.yellow + " --level=big_picture" + console.reset + "\n"
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
    parser.add_argument('-l', '--level', action='store',
                        default='fine_detail',
                        choices=['all', 'fine_detail', 'detail', 'component', 'big_picture'],
                        help='visibility level')
    return parser


def create_tree(level):
    root = py_trees.composites.Selector("Demo Dot Graphs %s" % level)
    first_blackbox = py_trees.composites.Sequence("BlackBox 1")
    first_blackbox.add_child(py_trees.behaviours.Running("Worker"))
    first_blackbox.add_child(py_trees.behaviours.Running("Worker"))
    first_blackbox.add_child(py_trees.behaviours.Running("Worker"))
    first_blackbox.blackbox_level = py_trees.common.BlackBoxLevel.BIG_PICTURE
    second_blackbox = py_trees.composites.Sequence("Blackbox 2")
    second_blackbox.add_child(py_trees.behaviours.Running("Worker"))
    second_blackbox.add_child(py_trees.behaviours.Running("Worker"))
    second_blackbox.add_child(py_trees.behaviours.Running("Worker"))
    second_blackbox.blackbox_level = py_trees.common.BlackBoxLevel.COMPONENT
    third_blackbox = py_trees.composites.Sequence("Blackbox 3")
    third_blackbox.add_child(py_trees.behaviours.Running("Worker"))
    third_blackbox.add_child(py_trees.behaviours.Running("Worker"))
    third_blackbox.add_child(py_trees.behaviours.Running("Worker"))
    third_blackbox.blackbox_level = py_trees.common.BlackBoxLevel.DETAIL
    root.add_child(first_blackbox)
    root.add_child(second_blackbox)
    first_blackbox.add_child(third_blackbox)
    return root


##############################################################################
# Main
##############################################################################

def main():
    """
    Entry point for the demo script.
    """
    args = command_line_argument_parser().parse_args()
    args.enum_level = py_trees.common.string_to_visibility_level(args.level)
    print(description())
    py_trees.logging.level = py_trees.logging.Level.DEBUG

    root = create_tree(args.level)
    py_trees.display.render_dot_tree(root, args.enum_level)

    if py_trees.utilities.which("xdot"):
        try:
            subprocess.call(["xdot", "demo_dot_graphs_%s.dot" % args.level])
        except KeyboardInterrupt:
            pass
    else:
        print("")
        console.logerror("No xdot viewer found, skipping display [hint: sudo apt install xdot]")
        print("")

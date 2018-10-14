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
   :module: py_trees.demos.stewardship
   :func: command_line_argument_parser
   :prog: py-trees-demo-tree-stewardship

.. graphviz:: dot/stewardship.dot

.. image:: images/tree_stewardship.gif
"""

##############################################################################
# Imports
##############################################################################

import argparse
import functools
import py_trees
import sys
import time

import py_trees.console as console

##############################################################################
# Classes
##############################################################################


def description(root):
    content = "A demonstration of tree stewardship.\n\n"
    content += "A slightly less trivial tree that uses a simple stdout pre-tick handler\n"
    content += "and both the debug and snapshot visitors for logging and displaying\n"
    content += "the state of the tree.\n"
    content += "\n"
    content += "EVENTS\n"
    content += "\n"
    content += " -  3 : sequence switches from running to success\n"
    content += " -  4 : selector's first child flicks to success once only\n"
    content += " -  8 : the fallback idler kicks in as everything else fails\n"
    content += " - 14 : the first child kicks in again, aborting a running sequence behind it\n"
    content += "\n"
    if py_trees.console.has_colours:
        banner_line = console.green + "*" * 79 + "\n" + console.reset
        s = "\n"
        s += banner_line
        s += console.bold_white + "Trees".center(79) + "\n" + console.reset
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
    parser = argparse.ArgumentParser(description=description(create_tree()),
                                     epilog=epilog(),
                                     formatter_class=argparse.RawDescriptionHelpFormatter,
                                     )
    group = parser.add_mutually_exclusive_group()
    group.add_argument('-r', '--render', action='store_true', help='render dot tree to file')
    group.add_argument('-i', '--interactive', action='store_true', help='pause and wait for keypress at each tick')
    return parser


def pre_tick_handler(behaviour_tree):
    """
    This prints a banner and will run immediately before every tick of the tree.

    Args:
        behaviour_tree (:class:`~py_trees.trees.BehaviourTree`): the tree custodian

    """
    print("\n--------- Run %s ---------\n" % behaviour_tree.count)


def post_tick_handler(snapshot_visitor, behaviour_tree):
    """
    Prints an ascii tree with the current snapshot status.
    """
    print("\n" + py_trees.display.ascii_tree(behaviour_tree.root,
                                             snapshot_information=snapshot_visitor))


def create_tree():
    every_n_success = py_trees.behaviours.SuccessEveryN("EveryN", 5)
    sequence = py_trees.composites.Sequence(name="Sequence")
    guard = py_trees.behaviours.Success("Guard")
    periodic_success = py_trees.behaviours.Periodic("Periodic", 3)
    finisher = py_trees.behaviours.Success("Finisher")
    sequence.add_child(guard)
    sequence.add_child(periodic_success)
    sequence.add_child(finisher)
    sequence.blackbox_level = py_trees.common.BlackBoxLevel.COMPONENT
    idle = py_trees.behaviours.Success("Idle")
    root = py_trees.composites.Selector(name="Demo Tree")
    root.add_child(every_n_success)
    root.add_child(sequence)
    root.add_child(idle)
    return root


##############################################################################
# Main
##############################################################################

def main():
    """
    Entry point for the demo script.
    """
    args = command_line_argument_parser().parse_args()
    py_trees.logging.level = py_trees.logging.Level.DEBUG
    tree = create_tree()
    print(description(tree))

    ####################
    # Rendering
    ####################
    if args.render:
        py_trees.display.render_dot_tree(tree)
        sys.exit()

    ####################
    # Tree Stewardship
    ####################
    behaviour_tree = py_trees.trees.BehaviourTree(tree)
    behaviour_tree.add_pre_tick_handler(pre_tick_handler)
    behaviour_tree.visitors.append(py_trees.visitors.DebugVisitor())
    snapshot_visitor = py_trees.visitors.SnapshotVisitor()
    behaviour_tree.add_post_tick_handler(functools.partial(post_tick_handler, snapshot_visitor))
    behaviour_tree.visitors.append(snapshot_visitor)
    behaviour_tree.setup(timeout=15)

    ####################
    # Tick Tock
    ####################
    if args.interactive:
        py_trees.console.read_single_keypress()
    while True:
        try:
            behaviour_tree.tick()
            if args.interactive:
                py_trees.console.read_single_keypress()
            else:
                time.sleep(0.5)
        except KeyboardInterrupt:
            break
    print("\n")

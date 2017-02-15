#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/stonier/py_trees_suite/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
.. module:: demos
   :synopsis: Functions and classes supporting demo scripts.

Oh my spaghettified magnificence,
Bless my noggin with a tickle from your noodly appendages!
"""

##############################################################################
# Imports
##############################################################################

import argparse
import sys
import time

from . import console
from . import display
from . import logging
from . import trees
from . import visitors

##############################################################################
# Main
##############################################################################


def main(root, show_usage):
    """
    Convenience function driving the demo scripts.
    :param root: a :class:`Behaviour <py_trees.behaviours.Behaviour>` representing the root of a behaviour tree
    :param func show_usage: function with empty signature which produces an argparse usage function
    """
    ####################
    # Logging
    ####################
    logging.level = logging.Level.DEBUG

    ####################
    # Arg Parsing
    ####################
    def _pre_tick_handler(behaviour_tree):
        print("\n--------- Run %s ---------\n" % behaviour_tree.count)

    show_usage()
    parser = argparse.ArgumentParser(description='Demo the behaviour trees', usage=show_usage())
    group = parser.add_mutually_exclusive_group()
    group.add_argument('-r', '--render', action='store_true', help='render dot tree to file')
    group.add_argument('-i', '--interactive', action='store_true', help='pause and wait for keypress at each tick')

    args = parser.parse_args(args=sys.argv[1:])

    ####################
    # Rendering
    ####################
    if args.render:
        display.render_dot_tree(root)
        return

    ####################
    # Tree
    ####################
    tree = trees.BehaviourTree(root)
    tree.visitors.append(visitors.DebugVisitor())
    snapshot_visitor = visitors.SnapshotVisitor()
    tree.visitors.append(snapshot_visitor)

    ####################
    # Tick Tock
    ####################
    while True:
        try:
            tree.tick(pre_tick_handler=_pre_tick_handler)
            print("\n" + display.ascii_tree(tree.root, snapshot_information=snapshot_visitor))
            if args.interactive:
                unused_result = console.read_single_keypress()
            else:
                time.sleep(0.5)
        except KeyboardInterrupt:
            break
    # tree.tick_tock(sleep_ms=500, number_of_iterations=py_trees.CONTINUOUS_TICK_TOCK, pre_tick_handler=pre_tick_handler)

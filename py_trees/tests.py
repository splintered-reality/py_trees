#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/splintered-reality/py_trees/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
Library of common methods for the tests.

Oh my spaghettified magnificence,
Bless my noggin with a tickle from your noodly appendages!
"""

##############################################################################
# Imports
##############################################################################

from . import blackboard
from . import console
from . import display

##############################################################################
# Methods
##############################################################################


def pre_tick_visitor(behaviour_tree):
    print("\n--------- Run %s ---------\n" % behaviour_tree.count)


def tick_tree(root,
              from_tick,
              to_tick,
              *,
              visitors=[],
              print_snapshot=False,
              print_blackboard=False
              ):
    print("\n================== Iteration {}-{} ==================\n".format(from_tick, to_tick))
    for i in range(from_tick, to_tick + 1):
        for visitor in visitors:
            visitor.initialise()
        print(("\n--------- Run %s ---------\n" % i))
        for node in root.tick():
            for visitor in visitors:
                node.visit(visitor)
    if print_snapshot:
        print(console.green + "\nAscii Tree Snapshot" + console.reset)
        print(display.unicode_tree(root=root, show_status=True))
    if print_blackboard:
        print(str(blackboard.Blackboard()))


def print_summary(nodes):
    print("\n--------- Summary ---------\n")
    for node in nodes:
        print("%s" % node)

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


def print_assert_banner():
    print(console.green + "\n--------- Assertions ---------\n" + console.reset)


def print_assert_details(text, expected, result):
    print(console.green + text +
          "." * (70 - len(text)) +
          console.cyan + "{}".format(expected) +
          console.yellow + " [{}]".format(result) +
          console.reset)


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
        print(console.green + "\nTree Snapshot" + console.reset)
        print(display.unicode_tree(root=root, show_status=True))
    if print_blackboard:
        print(display.unicode_blackboard())


def clear_blackboard():
    # Useful between tests
    blackboard.Blackboard.storage = {}
    blackboard.Blackboard.clients = {}
    blackboard.Blackboard.metadata = {}


def print_summary(nodes):
    print("\n--------- Summary ---------\n")
    for node in nodes:
        print("%s" % node)

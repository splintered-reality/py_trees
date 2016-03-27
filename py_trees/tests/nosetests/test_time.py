#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/gopher_crazy_hospital/py_trees/license/LICENSE
#

##############################################################################
# Imports
##############################################################################

# enable some python3 compatibility options:
# (unicode_literals not compatible with python2 uuid module)
from __future__ import absolute_import, print_function

from nose.tools import assert_raises
import py_trees
import rocon_console.console as console
import time

##############################################################################
# Logging Level
##############################################################################

py_trees.logging.level = py_trees.logging.Level.DEBUG
logger = py_trees.logging.get_logger("Nosetest")

##############################################################################
# Classes
##############################################################################

def pre_tick_visitor(behaviour_tree):
    print("\n--------- Run %s ---------\n" % behaviour_tree.count)


def tick_tree(tree, visitor, from_iteration, to_iteration, sleep_period):
    print("\n================== Iteration %s-%s ==================\n" % (from_iteration, to_iteration))
    for i in range(from_iteration, to_iteration + 1):
        print("\n--------- Run %s ---------\n" % i)
        for node in tree.tick():
            node.visit(visitor)
        time.sleep(sleep_period)

def print_summary(nodes):
    print("\n--------- Summary ---------\n")
    for node in nodes:
        print("%s" % node)

##############################################################################
# Tests
##############################################################################

def test_timeout_subtree():
    print(console.bold + "\n****************************************************************************************" + console.reset)
    print(console.bold + "* Timeout Subtree" + console.reset)
    print(console.bold + "****************************************************************************************\n" + console.reset)
    counter = py_trees.behaviours.Count(name="Counter", fail_until=0, running_until=10, success_until=20)
    root = py_trees.timers.create_timeout_subtree(counter, 1.0)
    py_trees.display.print_ascii_tree(root)
    visitor = py_trees.trees.DebugVisitor()
    tick_tree(root, visitor, 1, 5, 0.5)

    print("\n--------- Assertions ---------\n")
    print("root.status == py_trees.Status.RUNNING")
    assert(root.status == py_trees.Status.RUNNING)
    print("Number of Count Resets == 1")
    assert(counter.number_count_resets == 1)
    print("Count == 2")
    assert(counter.count == 2)

#     print("failure.status == py_trees.Status.FAILURE")
#     assert(failure.status == py_trees.Status.FAILURE)
#     print("goon.status == py_trees.Status.SUCCESS")
#     assert(goon.status == py_trees.Status.SUCCESS)

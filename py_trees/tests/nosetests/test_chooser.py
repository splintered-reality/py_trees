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


def tick_tree(tree, visitor, from_iteration, to_iteration):
    print("\n================== Iteration %s-%s ==================\n" % (from_iteration, to_iteration))
    for i in range(from_iteration, to_iteration + 1):
        print("\n--------- Run %s ---------\n" % i)
        for node in tree.tick():
            node.visit(visitor)

def print_summary(nodes):
    print("\n--------- Summary ---------\n")
    for node in nodes:
        print("%s" % node)

##############################################################################
# Tests
##############################################################################

def test_low_priority_runner():
    print(console.bold + "\n****************************************************************************************" + console.reset)
    print(console.bold + "* Low Priority Runner" + console.reset)
    print(console.bold + "****************************************************************************************\n" + console.reset)
    root = py_trees.composites.Chooser()
    failure = py_trees.behaviours.Failure("Failure")
    running = py_trees.behaviours.Running("Running")
    root.add_child(failure)
    root.add_child(running)
    py_trees.display.print_ascii_tree(root)
    visitor = py_trees.trees.DebugVisitor()
    tick_tree(root, visitor, 1, 1)

    print("\n--------- Assertions ---------\n")
    print("root.status == py_trees.Status.RUNNING")
    assert(root.status == py_trees.Status.RUNNING)
    print("failure.status == py_trees.Status.FAILURE")
    assert(failure.status == py_trees.Status.FAILURE)
    print("running.status == py_trees.Status.RUNNING")
    assert(running.status == py_trees.Status.RUNNING)

    tick_tree(root, visitor, 2, 2)

    print("\n--------- Assertions ---------\n")
    print("root.status == py_trees.Status.RUNNING")
    assert(root.status == py_trees.Status.RUNNING)
    print("failure.status == py_trees.Status.INVALID")
    assert(failure.status == py_trees.Status.INVALID)
    print("running.status == py_trees.Status.RUNNING")
    assert(running.status == py_trees.Status.RUNNING)

def test_low_priority_success():
    print(console.bold + "\n****************************************************************************************" + console.reset)
    print(console.bold + "* Low Priority Success" + console.reset)
    print(console.bold + "****************************************************************************************\n" + console.reset)
    root = py_trees.composites.Chooser()
    failure = py_trees.behaviours.Failure("Failure")
    success = py_trees.behaviours.Success("Success")
    root.add_child(failure)
    root.add_child(success)
    py_trees.display.print_ascii_tree(root)
    visitor = py_trees.trees.DebugVisitor()
    tick_tree(root, visitor, 1, 1)

    print("\n--------- Assertions ---------\n")
    print("root.status == py_trees.Status.SUCCESS")
    assert(root.status == py_trees.Status.SUCCESS)
    print("failure.status == py_trees.Status.FAILURE")
    assert(failure.status == py_trees.Status.FAILURE)
    print("success.status == py_trees.Status.SUCCESS")
    assert(success.status == py_trees.Status.SUCCESS)

    tick_tree(root, visitor, 2, 2)

    # make sure both children are ticked again (different to above)
    print("\n--------- Assertions ---------\n")
    print("root.status == py_trees.Status.SUCCESS")
    assert(root.status == py_trees.Status.SUCCESS)
    print("failure.status == py_trees.Status.FAILURE")
    assert(failure.status == py_trees.Status.FAILURE)
    print("success.status == py_trees.Status.SUCCESS")
    assert(success.status == py_trees.Status.SUCCESS)

def test_higher_priority_ignore():
    print(console.bold + "\n****************************************************************************************" + console.reset)
    print(console.bold + "* Ignore Higher Priority" + console.reset)
    print(console.bold + "****************************************************************************************\n" + console.reset)
    root = py_trees.composites.Chooser()
    ping_pong = py_trees.behaviours.SuccessEveryN("Ping Pong", 2)
    running = py_trees.behaviours.Running("Running")
    root.add_child(ping_pong)
    root.add_child(running)
    py_trees.display.print_ascii_tree(root)
    visitor = py_trees.trees.DebugVisitor()
    tick_tree(root, visitor, 1, 1)

    print("\n--------- Assertions ---------\n")
    print("root.status == py_trees.Status.RUNNING")
    assert(root.status == py_trees.Status.RUNNING)
    print("ping_pong.status == py_trees.Status.FAILURE")
    assert(ping_pong.status == py_trees.Status.FAILURE)
    print("running.status == py_trees.Status.RUNNING")
    assert(running.status == py_trees.Status.RUNNING)

    tick_tree(root, visitor, 2, 2)

    print("\n--------- Assertions ---------\n")
    print("root.status == py_trees.Status.RUNNING")
    assert(root.status == py_trees.Status.RUNNING)
    print("ping_pong.status == py_trees.Status.INVALID")
    assert(ping_pong.status == py_trees.Status.INVALID)  # got invalidated and didnt get ticked
    print("running.status == py_trees.Status.RUNNING")
    assert(running.status == py_trees.Status.RUNNING)


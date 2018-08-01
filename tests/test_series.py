#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/stonier/py_trees/devel/LICENSE
#

##############################################################################
# Imports
##############################################################################

# enable some python3 compatibility options:
# (unicode_literals not compatible with python2 uuid module)
from __future__ import absolute_import, print_function

import py_trees
import py_trees.console as console

##############################################################################
# Logging Level
##############################################################################

py_trees.logging.level = py_trees.logging.Level.DEBUG
logger = py_trees.logging.Logger("Nosetest")

##############################################################################
# Tests
##############################################################################

def test_series_suceess():
    print(console.bold + "\n****************************************************************************************" + console.reset)
    print(console.bold + "* Series Success" + console.reset)
    print(console.bold + "****************************************************************************************\n" + console.reset)
    root = py_trees.composites.Series('Series')
    success = py_trees.behaviours.Success("Success")

    root.add_child(success)
    py_trees.display.print_ascii_tree(root)
    visitor = py_trees.visitors.DebugVisitor()

    py_trees.tests.tick_tree(root, visitor, 1, 1)

    print("\n--------- Assertions ---------\n")
    print("root.status == py_trees.Status.SUCCESS")
    assert(root.status == py_trees.Status.SUCCESS)
    print("success.status == py_trees.Status.SUCCESS")
    assert(success.status == py_trees.Status.SUCCESS)


def test_series_failure():
    print(console.bold + "\n****************************************************************************************" + console.reset)
    print(console.bold + "* Series Failure" + console.reset)
    print(console.bold + "****************************************************************************************\n" + console.reset)
    root = py_trees.composites.Series('Series')
    failure = py_trees.behaviours.Failure("Failure")

    root.add_child(failure)
    py_trees.display.print_ascii_tree(root)
    visitor = py_trees.visitors.DebugVisitor()

    py_trees.tests.tick_tree(root, visitor, 1, 1)

    print("\n--------- Assertions ---------\n")
    print("root.status == py_trees.Status.FAILURE")
    assert(root.status == py_trees.Status.FAILURE)
    print("failure.status == py_trees.Status.FAILURE")
    assert(failure.status == py_trees.Status.FAILURE)


def test_series_running():
    print(console.bold + "\n****************************************************************************************" + console.reset)
    print(console.bold + "* Series Running" + console.reset)
    print(console.bold + "****************************************************************************************\n" + console.reset)
    root = py_trees.composites.Series('Series')
    success = py_trees.behaviours.Success("Success")
    running = py_trees.behaviours.Running("Running")

    root.add_child(success)
    root.add_child(running)
    py_trees.display.print_ascii_tree(root)
    visitor = py_trees.visitors.DebugVisitor()

    py_trees.tests.tick_tree(root, visitor, 1, 1)

    print("\n--------- Assertions ---------\n")
    print("root.status == py_trees.Status.RUNNING")
    assert(root.status == py_trees.Status.RUNNING)
    print("success.status == py_trees.Status.SUCCESS")
    assert(success.status == py_trees.Status.SUCCESS)
    print("running.status == py_trees.Status.RUNNING")
    assert(running.status == py_trees.Status.RUNNING)


def test_series_stop_future_children():
    print(console.bold + "\n****************************************************************************************" + console.reset)
    print(console.bold + "* Series Stop Future Children" + console.reset)
    print(console.bold + "****************************************************************************************\n" + console.reset)
    root = py_trees.composites.Series('Series')
    periodic = py_trees.behaviours.Periodic(name="periodic", n=0)
    running = py_trees.behaviours.Running("Running")

    root.add_child(periodic)
    root.add_child(running)
    py_trees.display.print_ascii_tree(root)
    visitor = py_trees.visitors.DebugVisitor()

    py_trees.tests.tick_tree(root, visitor, 1, 1)

    print("\n--------- Assertions ---------\n")
    print("root.status == py_trees.Status.RUNNING")
    assert(root.status == py_trees.Status.RUNNING)
    print("periodic.status == py_trees.Status.SUCCESS")
    assert(periodic.status == py_trees.Status.SUCCESS)
    print("running.status == py_trees.Status.RUNNING")
    assert(running.status == py_trees.Status.RUNNING)

    py_trees.tests.tick_tree(root, visitor, 2, 2)

    print("\n--------- Assertions ---------\n")
    print("root.status == py_trees.Status.FAILURE")
    assert(root.status == py_trees.Status.FAILURE)
    print("periodic.status == py_trees.Status.FAILURE")
    assert(periodic.status == py_trees.Status.FAILURE)
    print("running.status == py_trees.Status.INVALID")
    assert(running.status == py_trees.Status.INVALID)

    py_trees.tests.tick_tree(root, visitor, 3, 3)

    print("\n--------- Assertions ---------\n")
    print("root.status == py_trees.Status.RUNNING")
    assert(root.status == py_trees.Status.RUNNING)
    print("periodic.status == py_trees.Status.RUNNING")
    assert(periodic.status == py_trees.Status.RUNNING)
    print("running.status == py_trees.Status.INVALID")
    assert(running.status == py_trees.Status.INVALID)


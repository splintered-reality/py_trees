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


def test_parallel_failure():
    print(console.bold + "\n****************************************************************************************" + console.reset)
    print(console.bold + "* Parallel Failure" + console.reset)
    print(console.bold + "****************************************************************************************\n" + console.reset)
    root = py_trees.composites.Parallel("Parallel")
    failure = py_trees.behaviours.Failure("Failure")
    success = py_trees.behaviours.Success("Success")
    root.add_child(failure)
    root.add_child(success)
    py_trees.display.print_ascii_tree(root)
    visitor = py_trees.visitors.DebugVisitor()
    py_trees.tests.tick_tree(root, visitor, 1, 1)

    print("\n--------- Assertions ---------\n")
    print("root.status == py_trees.Status.FAILURE")
    assert(root.status == py_trees.Status.FAILURE)
    print("failure.status == py_trees.Status.FAILURE")
    assert(failure.status == py_trees.Status.FAILURE)
    print("success.status == py_trees.Status.SUCCESS")
    assert(success.status == py_trees.Status.SUCCESS)


def test_parallel_success():
    print(console.bold + "\n****************************************************************************************" + console.reset)
    print(console.bold + "* Parallel Success" + console.reset)
    print(console.bold + "****************************************************************************************\n" + console.reset)
    root = py_trees.composites.Parallel("Parallel")
    success1 = py_trees.behaviours.Success("Success1")
    success2 = py_trees.behaviours.Success("Success2")
    root.add_child(success1)
    root.add_child(success2)
    py_trees.display.print_ascii_tree(root)
    visitor = py_trees.visitors.DebugVisitor()
    py_trees.tests.tick_tree(root, visitor, 1, 1)

    print("\n--------- Assertions ---------\n")
    print("root.status == py_trees.Status.SUCCESS")
    assert(root.status == py_trees.Status.SUCCESS)
    print("success1.status == py_trees.Status.SUCCESS")
    assert(success1.status == py_trees.Status.SUCCESS)
    print("success2.status == py_trees.Status.SUCCESS")
    assert(success2.status == py_trees.Status.SUCCESS)


def test_parallel_running():
    print(console.bold + "\n****************************************************************************************" + console.reset)
    print(console.bold + "* Parallel Running" + console.reset)
    print(console.bold + "****************************************************************************************\n" + console.reset)
    root = py_trees.composites.Parallel("Parallel", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL)
    success_after_1 = py_trees.behaviours.Count(name="SuccessAfter1", fail_until=0, running_until=1, success_until=20)

    running = py_trees.behaviours.Running("Running")
    success_every_other = py_trees.behaviours.SuccessEveryN("SuccessEveryOther", 2)
    root.add_child(success_after_1)
    root.add_child(running)
    root.add_child(success_every_other)
    py_trees.display.print_ascii_tree(root)
    visitor = py_trees.visitors.DebugVisitor()
    py_trees.tests.tick_tree(root, visitor, 1, 1)

    print("\n--------- Assertions ---------\n")
    print("root.status == py_trees.Status.FAILURE")
    assert(root.status == py_trees.Status.FAILURE)
    print("success_after_1.status == py_trees.Status.FAILURE")
    assert(success_after_1.status == py_trees.Status.FAILURE)
    print("running.status == py_trees.Status.FAILURE")
    assert(running.status == py_trees.Status.FAILURE)
    print("success_every_other.status == py_trees.Status.FAILURE")
    assert(success_every_other.status == py_trees.Status.FAILURE)

    py_trees.tests.tick_tree(root, visitor, 2, 2)

    print("\n--------- Assertions ---------\n")
    print("root.status == py_trees.Status.RUNNING")
    assert(root.status == py_trees.Status.RUNNING)
    print("success_after_1.status == py_trees.Status.SUCCESS")
    assert(success_after_1.status == py_trees.Status.SUCCESS)
    print("running.status == py_trees.Status.RUNNING")
    assert(running.status == py_trees.Status.RUNNING)
    print("success_every_other.status == py_trees.Status.SUCCESS")
    assert(success_every_other.status == py_trees.Status.SUCCESS)


def test_parallel_success_on_one():
    print(console.bold + "\n****************************************************************************************" + console.reset)
    print(console.bold + "* Parallel Success on One" + console.reset)
    print(console.bold + "****************************************************************************************\n" + console.reset)
    print("")
    root = py_trees.composites.Parallel(name="Parallel", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
    running1 = py_trees.behaviours.Running("Running1")
    success = py_trees.behaviours.Success("Success")
    running2 = py_trees.behaviours.Running("Running2")
    root.add_child(running1)
    root.add_child(success)
    root.add_child(running2)
    py_trees.display.print_ascii_tree(root)
    visitor = py_trees.visitors.DebugVisitor()
    py_trees.tests.tick_tree(root, visitor, 1, 1)

    print("\n--------- Assertions ---------\n")
    print("All children get switched to success if one goes to success.")
    print("root.status == py_trees.Status.SUCCESS")
    assert(root.status == py_trees.Status.SUCCESS)
    print("running1.status == py_trees.Status.SUCCESS")
    assert(running1.status == py_trees.Status.SUCCESS)
    print("success.status == py_trees.Status.SUCCESS")
    assert(success.status == py_trees.Status.SUCCESS)
    print("running2.status == py_trees.Status.SUCCESS")
    assert(running2.status == py_trees.Status.SUCCESS)


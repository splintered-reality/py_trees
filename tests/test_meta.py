#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/stonier/py_trees/devel/LICENSE
#

##############################################################################
# Imports
##############################################################################

import py_trees
import py_trees.console as console
import time

##############################################################################
# Logging Level
##############################################################################

py_trees.logging.level = py_trees.logging.Level.DEBUG
logger = py_trees.logging.Logger("Nosetest")

##############################################################################
# Classes
##############################################################################


@py_trees.meta.failure_is_success
class MustGoOnRegardless(py_trees.behaviours.Failure):
    pass

##############################################################################
# Tests
##############################################################################


def test_failure_is_success_tree():
    console.banner("Failure is Success Tree")
    root = py_trees.composites.Selector(name="Root")
    failure = py_trees.behaviours.Failure(name="Failure")
    goon = MustGoOnRegardless(name="DontBeAfraidToBeTheGoon")
    root.add_child(failure)
    root.add_child(goon)
    py_trees.display.print_ascii_tree(root)
    visitor = py_trees.visitors.DebugVisitor()
    py_trees.tests.tick_tree(root, visitor, 1, 1)

    print("\n--------- Assertions ---------\n")
    print("root.status == py_trees.common.Status.SUCCESS")
    assert(root.status == py_trees.common.Status.SUCCESS)
    print("failure.status == py_trees.common.Status.FAILURE")
    assert(failure.status == py_trees.common.Status.FAILURE)
    print("goon.status == py_trees.common.Status.SUCCESS")
    assert(goon.status == py_trees.common.Status.SUCCESS)


def test_success_is_failure_tree():
    console.banner("Success is Failure Tree")
    root = py_trees.composites.Selector("Root")
    failure = py_trees.behaviours.Failure(name="Failure")
    going_down = py_trees.meta.success_is_failure(py_trees.behaviours.Success)(name="Going Down")
    root.add_child(failure)
    root.add_child(going_down)
    py_trees.display.print_ascii_tree(root)
    visitor = py_trees.visitors.DebugVisitor()
    py_trees.tests.tick_tree(root, visitor, 1, 1)

    print("\n--------- Assertions ---------\n")
    print("failure.status == py_trees.common.Status.FAILURE")
    assert(failure.status == py_trees.common.Status.FAILURE)
    print("going_down.status == py_trees.common.Status.FAILURE")
    assert(going_down.status == py_trees.common.Status.FAILURE)
    print("root.status == py_trees.common.Status.FAILURE")
    assert(root.status == py_trees.common.Status.FAILURE)


def test_inverter_tree():
    console.banner("Inverter Tree")
    root = py_trees.composites.Sequence(name="Root")
    selector = py_trees.composites.Selector(name="Selector")
    failure = py_trees.behaviours.Failure(name="Failure")
    failure2 = py_trees.meta.inverter(py_trees.behaviours.Success)(name="Failure2")
    success = py_trees.behaviours.Success(name="Success")
    success2 = py_trees.meta.inverter(py_trees.behaviours.Failure)(name="Success2")
    selector.add_child(failure)
    selector.add_child(failure2)
    selector.add_child(success)
    root.add_child(selector)
    root.add_child(success2)
    py_trees.display.print_ascii_tree(root)
    visitor = py_trees.visitors.DebugVisitor()
    py_trees.tests.tick_tree(root, visitor, 1, 1)

    print("\n--------- Assertions ---------\n")
    print("success.status == py_trees.common.Status.SUCCESS")
    assert(success.status == py_trees.common.Status.SUCCESS)
    print("success2.status == py_trees.common.Status.SUCCESS")
    assert(success2.status == py_trees.common.Status.SUCCESS)
    print("root.status == py_trees.common.Status.SUCCESS")
    assert(root.status == py_trees.common.Status.SUCCESS)
    print("failure.status == py_trees.common.Status.FAILURE")
    assert(failure.status == py_trees.common.Status.FAILURE)
    print("failure2.status == py_trees.common.Status.FAILURE")
    assert(failure2.status == py_trees.common.Status.FAILURE)


def test_running_is_failure_tree():
    console.banner("Running is Failure Tree")
    root = py_trees.composites.Selector(name="Root")
    running = py_trees.meta.running_is_failure(py_trees.behaviours.Running)(name="Running")
    failure = py_trees.meta.running_is_failure(py_trees.behaviours.Failure)(name="Failure")
    success = py_trees.meta.running_is_failure(py_trees.behaviours.Success)(name="Success")
    root.add_child(running)
    root.add_child(failure)
    root.add_child(success)
    py_trees.display.print_ascii_tree(root)
    visitor = py_trees.visitors.DebugVisitor()
    py_trees.tests.tick_tree(root, visitor, 1, 1)

    print("\n--------- Assertions ---------\n")
    print("running.status == py_trees.common.Status.FAILURE")
    assert(running.status == py_trees.common.Status.FAILURE)
    print("failure.status == py_trees.common.Status.FAILURE")
    assert(failure.status == py_trees.common.Status.FAILURE)
    print("success.status == py_trees.common.Status.SUCCESS")
    assert(success.status == py_trees.common.Status.SUCCESS)
    print("root.status == py_trees.common.Status.SUCCESS")
    assert(root.status == py_trees.common.Status.SUCCESS)


def test_inverter_sequence():
    console.banner("Inverter Sequence Tree")
    root = py_trees.meta.inverter(py_trees.composites.Sequence)(name="Root")
    selector = py_trees.composites.Selector(name="Selector")
    failure = py_trees.behaviours.Failure(name="Failure")
    success = py_trees.behaviours.Success(name="Success")
    selector.add_child(failure)
    selector.add_child(success)
    success2 = py_trees.behaviours.Success(name="Success2")
    root.add_child(selector)
    root.add_child(success2)
    py_trees.display.print_ascii_tree(root)
    visitor = py_trees.visitors.DebugVisitor()
    py_trees.tests.tick_tree(root, visitor, 1, 1)

    print("\n--------- Assertions ---------\n")
    print("root.status == py_trees.common.Status.FAILURE")
    assert(root.status == py_trees.common.Status.FAILURE)
    print("success.status == py_trees.common.Status.SUCCESS")
    assert(success.status == py_trees.common.Status.SUCCESS)
    print("success2.status == py_trees.common.Status.SUCCESS")
    assert(success2.status == py_trees.common.Status.SUCCESS)
    print("failure.status == py_trees.common.Status.FAILURE")
    assert(failure.status == py_trees.common.Status.FAILURE)
    print("selector.status == py_trees.common.Status.SUCCESS")
    assert(selector.status == py_trees.common.Status.SUCCESS)


def test_timeout():
    console.banner("Timeout")
    root = py_trees.meta.timeout(py_trees.behaviours.Running, 0.2)(name="Success w/ Timeout")
    py_trees.display.print_ascii_tree(root)
    visitor = py_trees.visitors.DebugVisitor()
    py_trees.tests.tick_tree(root, visitor, 1, 1)

    print("\n--------- Assertions ---------\n")
    print("root.status == py_trees.common.Status.RUNNING")
    print("root.status %s" % root.status)
    assert(root.status == py_trees.common.Status.RUNNING)

    time.sleep(0.3)
    py_trees.tests.tick_tree(root, visitor, 1, 1)

    print("\n--------- Assertions ---------\n")
    print("root.status == py_trees.common.Status.FAILURE")
    assert(root.status == py_trees.common.Status.FAILURE)


def test_condition():
    console.banner("Condition")

    Conditional = py_trees.meta.condition(py_trees.behaviours.Count, py_trees.common.Status.SUCCESS)
    condition = Conditional(name="D", fail_until=2, running_until=2, success_until=10, reset=False)

    visitor = py_trees.visitors.DebugVisitor()
    py_trees.tests.tick_tree(condition, visitor, 1, 1)

    print("\n--------- Assertions ---------\n")
    print("condition.original.status == py_trees.common.Status.FAILURE")
    assert(condition.original.status == py_trees.common.Status.FAILURE)
    print("condition.status == py_trees.common.Status.RUNNING")
    assert(condition.status == py_trees.common.Status.RUNNING)

    py_trees.tests.tick_tree(condition, visitor, 2, 2)

    print("\n--------- Assertions ---------\n")
    print("condition.original.status == py_trees.common.Status.FAILURE")
    assert(condition.original.status == py_trees.common.Status.FAILURE)
    print("condition.status == py_trees.common.Status.RUNNING")
    assert(condition.status == py_trees.common.Status.RUNNING)

    py_trees.tests.tick_tree(condition, visitor, 3, 3)

    print("\n--------- Assertions ---------\n")
    print("condition.original.status == py_trees.common.Status.SUCCESS")
    assert(condition.original.status == py_trees.common.Status.SUCCESS)
    print("condition.status == py_trees.common.Status.SUCCESS")
    assert(condition.status == py_trees.common.Status.SUCCESS)

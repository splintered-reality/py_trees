#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/splintered-reality/py_trees/devel/LICENSE
#

##############################################################################
# Imports
##############################################################################

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


def test_low_priority_runner():
    console.banner("Low Priority Runner")
    root = py_trees.composites.Chooser()
    failure = py_trees.behaviours.Failure("Failure")
    running = py_trees.behaviours.Running("Running")
    root.add_child(failure)
    root.add_child(running)
    print(py_trees.display.unicode_tree(root))
    visitor = py_trees.visitors.DebugVisitor()
    py_trees.tests.tick_tree(root, 1, 1, visitors=[visitor])

    print("\n--------- Assertions ---------\n")
    print("root.status == py_trees.common.Status.RUNNING")
    assert(root.status == py_trees.common.Status.RUNNING)
    print("failure.status == py_trees.common.Status.FAILURE")
    assert(failure.status == py_trees.common.Status.FAILURE)
    print("running.status == py_trees.common.Status.RUNNING")
    assert(running.status == py_trees.common.Status.RUNNING)

    py_trees.tests.tick_tree(root, 2, 2, visitors=[visitor])

    print("\n--------- Assertions ---------\n")
    print("root.status == py_trees.common.Status.RUNNING")
    assert(root.status == py_trees.common.Status.RUNNING)
    print("failure.status == py_trees.common.Status.INVALID")
    assert(failure.status == py_trees.common.Status.INVALID)
    print("running.status == py_trees.common.Status.RUNNING")
    assert(running.status == py_trees.common.Status.RUNNING)


def test_low_priority_success():
    console.banner("Low Priority Success")
    root = py_trees.composites.Chooser()
    failure = py_trees.behaviours.Failure("Failure")
    success = py_trees.behaviours.Success("Success")
    root.add_child(failure)
    root.add_child(success)
    print(py_trees.display.unicode_tree(root))
    visitor = py_trees.visitors.DebugVisitor()
    py_trees.tests.tick_tree(root, 1, 1, visitors=[visitor])

    print("\n--------- Assertions ---------\n")
    print("root.status == py_trees.common.Status.SUCCESS")
    assert(root.status == py_trees.common.Status.SUCCESS)
    print("failure.status == py_trees.common.Status.FAILURE")
    assert(failure.status == py_trees.common.Status.FAILURE)
    print("success.status == py_trees.common.Status.SUCCESS")
    assert(success.status == py_trees.common.Status.SUCCESS)

    py_trees.tests.tick_tree(root, 2, 2, visitors=[visitor])

    # make sure both children are ticked again (different to above)
    print("\n--------- Assertions ---------\n")
    print("root.status == py_trees.common.Status.SUCCESS")
    assert(root.status == py_trees.common.Status.SUCCESS)
    print("failure.status == py_trees.common.Status.FAILURE")
    assert(failure.status == py_trees.common.Status.FAILURE)
    print("success.status == py_trees.common.Status.SUCCESS")
    assert(success.status == py_trees.common.Status.SUCCESS)


def test_higher_priority_ignore():
    console.banner("Ignore Higher Priority")
    root = py_trees.composites.Chooser()
    ping_pong = py_trees.behaviours.SuccessEveryN("Ping Pong", 2)
    running = py_trees.behaviours.Running("Running")
    root.add_child(ping_pong)
    root.add_child(running)
    print(py_trees.display.unicode_tree(root))
    visitor = py_trees.visitors.DebugVisitor()
    py_trees.tests.tick_tree(root, 1, 1, visitors=[visitor])

    print("\n--------- Assertions ---------\n")
    print("root.status == py_trees.common.Status.RUNNING")
    assert(root.status == py_trees.common.Status.RUNNING)
    print("ping_pong.status == py_trees.common.Status.FAILURE")
    assert(ping_pong.status == py_trees.common.Status.FAILURE)
    print("running.status == py_trees.common.Status.RUNNING")
    assert(running.status == py_trees.common.Status.RUNNING)

    py_trees.tests.tick_tree(root, 2, 2, visitors=[visitor])

    print("\n--------- Assertions ---------\n")
    print("root.status == py_trees.common.Status.RUNNING")
    assert(root.status == py_trees.common.Status.RUNNING)
    print("ping_pong.status == py_trees.common.Status.INVALID")
    assert(ping_pong.status == py_trees.common.Status.INVALID)  # got invalidated and didnt get ticked
    print("running.status == py_trees.common.Status.RUNNING")
    assert(running.status == py_trees.common.Status.RUNNING)

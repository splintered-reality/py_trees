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


def test_single_behaviour():
    console.banner("Single Behaviour")
    failure = py_trees.behaviours.Failure(name="Failure")
    running = py_trees.behaviours.Running(name="Running")
    success = py_trees.behaviours.Success(name="Success")
    print("\nAll status' except INVALID are permissable.")
    print("\n--------- Assertions ---------\n")
    print("failure.tip()[before tick]......None [{}]".format(failure.tip()))
    assert(failure.tip() is None)
    print("")
    failure.tick_once()
    running.tick_once()
    success.tick_once()
    print("")
    print("failure.tip()...................Failure [{}]".format(failure.tip().name))
    assert(failure.tip() is failure)
    print("running.tip()...................Running [{}]".format(running.tip().name))
    assert(running.tip() is running)
    print("success.tip()...................Success [{}]".format(success.tip().name))
    assert(success.tip() is success)


def test_sequence():
    console.banner("Sequence")
    print("\n--------- Assertions ---------\n")
    root = py_trees.composites.Sequence(name="Root")
    running = py_trees.behaviours.Running(name="Running")
    success = py_trees.behaviours.Success(name="Success")
    root.add_children([success, running])
    py_trees.tests.tick_tree(root, 1, 1, print_snapshot=True)
    print("root.tip()...................running [{}]".format(root.tip()))
    assert(root.tip() is running)

    root = py_trees.composites.Sequence(name="Root")
    success_one = py_trees.behaviours.Success(name="Success 1")
    success_two = py_trees.behaviours.Success(name="Success 2")
    root.add_children([success_one, success_two])
    py_trees.tests.tick_tree(root, 1, 1, print_snapshot=True)

    print("root.tip()...................Success 2 [{}]".format(root.tip().name))
    assert(root.tip() is success_two)

    root = py_trees.composites.Sequence(name="Root")
    failure = py_trees.behaviours.Failure(name="Failure")
    root.add_children([failure])
    py_trees.tests.tick_tree(root, 1, 1, print_snapshot=True)

    print("root.tip()...................Failure [{}]".format(root.tip().name))
    assert(root.tip() is failure)

    root = py_trees.composites.Sequence(name="Root")
    py_trees.tests.tick_tree(root, 1, 1, print_snapshot=True)
    print("root.tip()...................Root [{}]".format(root.tip().name))
    assert(root.tip() is root)


def test_selector():
    console.banner("Selector")
    print("\n--------- Assertions ---------\n")
    root = py_trees.composites.Selector(name="Root")
    failure = py_trees.behaviours.Failure(name="Failure")
    running = py_trees.behaviours.Running(name="Running")
    root.add_children([failure, running])
    py_trees.tests.tick_tree(root, 1, 1, print_snapshot=True)
    print("root.tip()...................running [{}]".format(root.tip().name))
    assert(root.tip() is running)

    root = py_trees.composites.Selector(name="Root")
    failure = py_trees.behaviours.Failure(name="Failure")
    success = py_trees.behaviours.Success(name="Success")
    root.add_children([failure, success])
    py_trees.tests.tick_tree(root, 1, 1, print_snapshot=True)
    print("root.tip()...................success [{}]".format(root.tip().name))
    assert(root.tip() is success)

    root = py_trees.composites.Selector(name="Root")
    running = py_trees.behaviours.Running(name="Running")
    failure = py_trees.behaviours.Failure(name="Failure")
    root.add_children([running, failure])
    py_trees.tests.tick_tree(root, 1, 1, print_snapshot=True)
    print("root.tip()...................running [{}]".format(root.tip().name))
    assert(root.tip() is running)

    root = py_trees.composites.Selector(name="Root")
    success = py_trees.behaviours.Success(name="Success")
    failure = py_trees.behaviours.Failure(name="Failure")
    root.add_children([success, failure])
    py_trees.tests.tick_tree(root, 1, 1, print_snapshot=True)
    print("root.tip()...................success [{}]".format(root.tip().name))
    assert(root.tip() is success)

    root = py_trees.composites.Selector(name="Root")
    failure = py_trees.behaviours.Failure(name="Failure 1")
    failure_two = py_trees.behaviours.Failure(name="Failure 2")
    root.add_children([failure, failure_two])
    py_trees.tests.tick_tree(root, 1, 1, print_snapshot=True)
    print("root.tip()...................Failure 2 [{}]".format(root.tip().name))
    assert(root.tip() is failure_two)

    root = py_trees.composites.Selector(name="Root")
    fail_then_run = py_trees.behaviours.Count(name="Fail Then Run", fail_until=1, running_until=10)
    running = py_trees.behaviours.Running(name="Running")
    root.add_children([fail_then_run, running])
    py_trees.tests.tick_tree(root, 1, 1, print_snapshot=True)
    print("root.tip()...................Running [{}]".format(root.tip().name))
    assert(root.tip() is running)
    py_trees.tests.tick_tree(root, 2, 2, print_snapshot=True)
    print("root.tip()...................Fail Then Run [{}]".format(root.tip().name))
    assert(root.tip() is fail_then_run)

    root = py_trees.composites.Selector(name="Root")
    py_trees.tests.tick_tree(root, 1, 1, print_snapshot=True)
    print("root.tip()...................Root [{}]".format(root.tip().name))
    assert(root.tip() is root)


def test_decorator():
    console.banner("Decorators")
    print("\n--------- Assertions ---------\n")

    # Decorators are the exception, when they reach SUCCESS||FAILURE and
    # the child is RUNNING they invalidate their children, so look to the
    # decorator, not the child!
    child = py_trees.behaviours.Running(name="Child")
    root = py_trees.decorators.RunningIsFailure(name="Decorator", child=child)
    py_trees.tests.tick_tree(root, 1, 1, print_snapshot=True)
    print("root.tip()...................Decorator [{}]".format(root.tip().name))
    assert(root.tip() is root)

    child = py_trees.behaviours.Success(name="Child")
    root = py_trees.decorators.RunningIsSuccess(name="Decorator", child=child)
    py_trees.tests.tick_tree(root, 1, 1, print_snapshot=True)
    print("root.tip()...................Child [{}]".format(root.tip().name))
    assert(root.tip() is child)
    child = py_trees.behaviours.Running(name="Child")

    child = py_trees.behaviours.Running(name="Child")
    root = py_trees.decorators.SuccessIsFailure(name="Decorator", child=child)
    py_trees.tests.tick_tree(root, 1, 1, print_snapshot=True)
    print("root.tip()...................Child [{}]".format(root.tip().name))
    assert(root.tip() is child)


def test_parallel():
    console.banner("Parallel")
    root = py_trees.composites.Parallel(name="Root", policy=py_trees.common.ParallelPolicy.SuccessOnAll())
    failure = py_trees.behaviours.Failure(name="Failure")
    running = py_trees.behaviours.Running(name="Running")
    root.add_children([failure, running])
    py_trees.tests.tick_tree(root, 1, 1, print_snapshot=True)
    # running node is invalidated because failure failed
    print("\n----- Assertions (OnAll) -----\n")
    print("root.tip()...................failure [{}]".format(root.tip().name))
    assert(root.tip() is failure)

    root = py_trees.composites.Parallel(name="Root", policy=py_trees.common.ParallelPolicy.SuccessOnAll())
    failure = py_trees.behaviours.Failure(name="Failure")
    success = py_trees.behaviours.Success(name="Success")
    root.add_children([failure, success])
    py_trees.tests.tick_tree(root, 1, 1, print_snapshot=True)
    print("\n----- Assertions (OnAll) -----\n")
    print("root.tip()...................failure [{}]".format(root.tip().name))
    assert(root.tip() is failure)

    root = py_trees.composites.Parallel(name="Root", policy=py_trees.common.ParallelPolicy.SuccessOnOne())
    failure = py_trees.behaviours.Failure(name="Failure")
    success = py_trees.behaviours.Success(name="Success")
    root.add_children([failure, success])
    py_trees.tests.tick_tree(root, 1, 1, print_snapshot=True)
    print("\n----- Assertions (OnOne) -----\n")
    print("root.tip()...................failure [{}]".format(root.tip().name))
    assert(root.tip() is failure)

    root = py_trees.composites.Parallel(name="Root", policy=py_trees.common.ParallelPolicy.SuccessOnAll())
    success = py_trees.behaviours.Success(name="Success")
    running = py_trees.behaviours.Running(name="Running")
    root.add_children([success, running])
    py_trees.tests.tick_tree(root, 1, 1, print_snapshot=True)
    # running node is tip because it was updated last
    print("\n----- Assertions (OnAll) -----\n")
    print("root.tip()...................running [{}]".format(root.tip().name))
    assert(root.tip() is running)

    root = py_trees.composites.Parallel(name="Root", policy=py_trees.common.ParallelPolicy.SuccessOnOne())
    success = py_trees.behaviours.Success(name="Success")
    running = py_trees.behaviours.Running(name="Running")
    root.add_children([success, running])
    py_trees.tests.tick_tree(root, 1, 1, print_snapshot=True)
    # success is tip because it flipped it's parent to success
    print("\n----- Assertions (OnOne) -----\n")
    print("root.tip()...................success [{}]".format(root.tip().name))
    assert(root.tip() is success)

    root = py_trees.composites.Parallel(name="Root", policy=py_trees.common.ParallelPolicy.SuccessOnAll())
    running = py_trees.behaviours.Running(name="Running 1")
    running_two = py_trees.behaviours.Running(name="Running 2")
    root.add_children([running, running_two])
    py_trees.tests.tick_tree(root, 1, 1, print_snapshot=True)
    # running two node is tip because it was updated last
    print("\n----- Assertions (OnAll) -----\n")
    print("root.tip()...................Running 2 [{}]".format(root.tip().name))
    assert(root.tip() is running_two)

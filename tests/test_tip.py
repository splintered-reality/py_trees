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

from nose.tools import assert_raises
import time

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


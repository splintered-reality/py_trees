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
import time

##############################################################################
# Logging Level
##############################################################################

py_trees.logging.level = py_trees.logging.Level.DEBUG
logger = py_trees.logging.Logger("Nosetest")


##############################################################################
# Classes
##############################################################################

class StateHolder(py_trees.behaviour.Behaviour):
    def __init__(self, name="State Holder"):
        super(StateHolder, self).__init__(name=name)
        self.setup_called = False

    def setup(self, timeout):
        self.logger.debug("%s.setup()" % (self.__class__.__name__))
        self.setup_called = True
        return True

    def update(self):
        return py_trees.Status.SUCCESS


##############################################################################
# Tests
##############################################################################

def test_failure_is_success_tree():
    print(console.bold + "\n****************************************************************************************" + console.reset)
    print(console.bold + "* Failure is Success Tree" + console.reset)
    print(console.bold + "****************************************************************************************\n" + console.reset)
    root = py_trees.Selector("Root")
    failure = py_trees.behaviours.Failure("Failure")
    goon = py_trees.decorators.FailureIsSuccess(py_trees.behaviours.Failure(name="Failure"), "DontBeAfraidToBeTheGoon")
    root.add_child(failure)
    root.add_child(goon)
    py_trees.display.print_ascii_tree(root)
    visitor = py_trees.visitors.DebugVisitor()
    py_trees.tests.tick_tree(root, visitor, 1, 1)

    print("\n--------- Assertions ---------\n")
    print("root.status == py_trees.Status.SUCCESS")
    assert(root.status == py_trees.Status.SUCCESS)
    print("failure.status == py_trees.Status.FAILURE")
    assert(failure.status == py_trees.Status.FAILURE)
    print("goon.decorated.status == py_trees.Status.FAILURE")
    assert(goon.decorated.status == py_trees.Status.FAILURE)
    print("goon.status == py_trees.Status.SUCCESS")
    assert(goon.status == py_trees.Status.SUCCESS)


def test_success_is_failure_tree():
    print(console.bold + "\n****************************************************************************************" + console.reset)
    print(console.bold + "* Success is Failure Tree" + console.reset)
    print(console.bold + "****************************************************************************************\n" + console.reset)
    root = py_trees.Selector("Root")
    failure = py_trees.behaviours.Failure("Failure")
    success = py_trees.behaviours.Success(name="Success")
    going_down = py_trees.decorators.SuccessIsFailure(success, "Going Down")
    root.add_child(failure)
    root.add_child(going_down)
    py_trees.display.print_ascii_tree(root)
    visitor = py_trees.visitors.DebugVisitor()
    py_trees.tests.tick_tree(root, visitor, 1, 1)

    print("\n--------- Assertions ---------\n")
    print("failure.status == py_trees.Status.FAILURE")
    assert(failure.status == py_trees.Status.FAILURE)
    print("going_down.decorated.status == py_trees.Status.SUCCESS")
    assert(going_down.decorated.status == py_trees.Status.SUCCESS)
    print("going_down.status == py_trees.Status.FAILURE")
    assert(going_down.status == py_trees.Status.FAILURE)
    print("root.status == py_trees.Status.FAILURE")
    assert(root.status == py_trees.Status.FAILURE)


def test_inverter_tree():
    print(console.bold + "\n****************************************************************************************" + console.reset)
    print(console.bold + "* Inverter Tree" + console.reset)
    print(console.bold + "****************************************************************************************\n" + console.reset)
    root = py_trees.Sequence("Root")
    selector = py_trees.Selector("Selector")
    failure = py_trees.behaviours.Failure("Failure")
    failure2 = py_trees.decorators.Inverter(py_trees.behaviours.Success(name="Success"), "Failure2")
    success = py_trees.behaviours.Success("Success")
    success2 = py_trees.decorators.Inverter(py_trees.behaviours.Failure(name="Failure"), "Success2")
    selector.add_child(failure)
    selector.add_child(failure2)
    selector.add_child(success)
    root.add_child(selector)
    root.add_child(success2)
    py_trees.display.print_ascii_tree(root)
    visitor = py_trees.visitors.DebugVisitor()
    py_trees.tests.tick_tree(root, visitor, 1, 1)

    print("\n--------- Assertions ---------\n")
    print("success.status == py_trees.Status.SUCCESS")
    assert(success.status == py_trees.Status.SUCCESS)
    print("success2.decorated.status == py_trees.Status.FAILURE")
    assert(success2.decorated.status == py_trees.Status.FAILURE)
    print("success2.status == py_trees.Status.SUCCESS")
    assert(success2.status == py_trees.Status.SUCCESS)
    print("root.status == py_trees.Status.SUCCESS")
    assert(root.status == py_trees.Status.SUCCESS)
    print("failure.status == py_trees.Status.FAILURE")
    assert(failure.status == py_trees.Status.FAILURE)
    print("failure2.decorated.status == py_trees.Status.SUCCESS")
    print("failure2.decorated.status == py_trees.Status.SUCCESS")
    print("failure2.status == py_trees.Status.FAILURE")
    assert(failure2.status == py_trees.Status.FAILURE)


def test_running_is_failure_tree():
    print(console.bold + "\n****************************************************************************************" + console.reset)
    print(console.bold + "* Running is Failure Tree" + console.reset)
    print(console.bold + "****************************************************************************************\n" + console.reset)
    root = py_trees.Selector("Root")
    running = py_trees.decorators.RunningIsFailure(py_trees.behaviours.Running(name="Running"), "Running")
    failure = py_trees.decorators.RunningIsFailure(py_trees.behaviours.Failure(name="Failure"), "Failure")
    success = py_trees.decorators.RunningIsFailure(py_trees.behaviours.Success(name="Success"), "Success")
    root.add_child(running)
    root.add_child(failure)
    root.add_child(success)
    py_trees.display.print_ascii_tree(root)
    visitor = py_trees.visitors.DebugVisitor()
    py_trees.tests.tick_tree(root, visitor, 1, 1)

    print("\n--------- Assertions ---------\n")
    print("running.decorated.status == py_trees.Status.RUNNING")
    assert(running.decorated.status == py_trees.Status.RUNNING)
    print("running.status == py_trees.Status.FAILURE")
    assert(running.status == py_trees.Status.FAILURE)
    print("failure.decorated.status == py_trees.Status.FAILURE")
    assert("failure.decorated.status == py_trees.Status.FAILURE")
    print("failure.status == py_trees.Status.FAILURE")
    assert(failure.status == py_trees.Status.FAILURE)
    print("success.decorated.status == py_trees.Status.SUCCESS")
    assert(success.decorated.status == py_trees.Status.SUCCESS)
    print("success.status == py_trees.Status.SUCCESS")
    assert(success.status == py_trees.Status.SUCCESS)
    print("root.status == py_trees.Status.SUCCESS")
    assert(root.status == py_trees.Status.SUCCESS)


def test_inverter_sequence():
    print(console.bold + "\n****************************************************************************************" + console.reset)
    print(console.bold + "* Inverter Sequence Tree" + console.reset)
    print(console.bold + "****************************************************************************************\n" + console.reset)
    root_decorated = py_trees.Sequence()
    root = py_trees.decorators.Inverter(root_decorated, "Root")
    selector = py_trees.Selector("Selector")
    failure = py_trees.behaviours.Failure("Failure")
    success = py_trees.behaviours.Success("Success")
    selector.add_child(failure)
    selector.add_child(success)
    success2 = py_trees.behaviours.Success("Success2")
    root_decorated.add_child(selector)
    root_decorated.add_child(success2)
    py_trees.display.print_ascii_tree(root)
    visitor = py_trees.visitors.DebugVisitor()
    py_trees.tests.tick_tree(root, visitor, 1, 1)

    print("\n--------- Assertions ---------\n")
    print("root_decorated is root.decorated")
    assert("root_decorated is root.decorated")
    print("root_decorated.status == py_trees.Status.SUCCESS")
    assert(root_decorated.status == py_trees.Status.SUCCESS)
    print("root.decorated.status == py_trees.Status.SUCCESS")
    assert(root.decorated.status == py_trees.Status.SUCCESS)
    print("root.status == py_trees.Status.FAILURE")
    assert(root.status == py_trees.Status.FAILURE)
    print("success.status == py_trees.Status.SUCCESS")
    assert(success.status == py_trees.Status.SUCCESS)
    print("success2.status == py_trees.Status.SUCCESS")
    assert(success2.status == py_trees.Status.SUCCESS)
    print("failure.status == py_trees.Status.FAILURE")
    assert(failure.status == py_trees.Status.FAILURE)
    print("selector.status == py_trees.Status.SUCCESS")
    assert(selector.status == py_trees.Status.SUCCESS)


def test_timeout():
    print(console.bold + "\n****************************************************************************************" + console.reset)
    print(console.bold + "* Timeout" + console.reset)
    print(console.bold + "****************************************************************************************\n" + console.reset)
    root = py_trees.decorators.Timeout(py_trees.behaviours.Running(name="Failure"), 0.2) # , name="Success w/ Timeout")
    py_trees.display.print_ascii_tree(root)
    visitor = py_trees.visitors.DebugVisitor()
    py_trees.tests.tick_tree(root, visitor, 1, 1)

    print("\n--------- Assertions ---------\n")
    print("root.status == py_trees.Status.RUNNING")
    print("root.status %s" % root.status)
    assert(root.status == py_trees.Status.RUNNING)
    print("root.decorated.status == py_trees.Status.RUNNING")
    assert(root.decorated.status == py_trees.Status.RUNNING)

    time.sleep(0.3)
    py_trees.tests.tick_tree(root, visitor, 1, 1)

    print("\n--------- Assertions ---------\n")
    print("root.status == py_trees.Status.FAILURE")
    assert(root.status == py_trees.Status.FAILURE)
    print("root.decorated.status == py_trees.Status.INVALID")
    assert(root.decorated.status == py_trees.Status.INVALID)


def test_condition():
    print(console.bold + "\n****************************************************************************************" + console.reset)
    print(console.bold + "* Condition" + console.reset)
    print(console.bold + "****************************************************************************************\n" + console.reset)

    counter = py_trees.behaviours.Count(name="D", fail_until=2, running_until=2, success_until=10, reset=False)
    condition = py_trees.decorators.Condition(counter, py_trees.Status.SUCCESS)
    py_trees.display.print_ascii_tree(condition)
    visitor = py_trees.visitors.DebugVisitor()
    py_trees.tests.tick_tree(condition, visitor, 1, 1)

    print("\n--------- Assertions ---------\n")
    print("condition.decorated.status == py_trees.Status.FAILURE")
    assert(condition.decorated.status == py_trees.Status.FAILURE)
    print("condition.status == py_trees.Status.RUNNING")
    assert(condition.status == py_trees.Status.RUNNING)

    py_trees.tests.tick_tree(condition, visitor, 2, 2)

    print("\n--------- Assertions ---------\n")
    print("condition.decorated.status == py_trees.Status.FAILURE")
    assert(condition.decorated.status == py_trees.Status.FAILURE)
    print("condition.status == py_trees.Status.RUNNING")
    assert(condition.status == py_trees.Status.RUNNING)

    py_trees.tests.tick_tree(condition, visitor, 3, 3)

    print("\n--------- Assertions ---------\n")
    print("condition.decorated.status == py_trees.Status.SUCCESS")
    assert(condition.decorated.status == py_trees.Status.SUCCESS)
    print("condition.status == py_trees.Status.SUCCESS")
    assert(condition.status == py_trees.Status.SUCCESS)


def test_new_decorator():
    print(console.bold + "\n****************************************************************************************" + console.reset)
    print(console.bold + "* New Decorator" + console.reset)
    print(console.bold + "****************************************************************************************\n" + console.reset)

    state_holder = StateHolder()
    decorator = py_trees.decorators.Decorator(state_holder)

    visitor = py_trees.visitors.DebugVisitor()
    py_trees.tests.tick_tree(decorator, visitor, 1, 1)

    print("\n--------- Assertions ---------\n")
    print("decorator.setup() is True")
    assert(decorator.setup(1) is True)
    print("state_holder.setup_called is True")
    assert(state_holder.setup_called is True)

    print("decorator.status == py_trees.Status.SUCCESS")
    assert(decorator.status == py_trees.Status.SUCCESS)
    print("state_holder.status == py_trees.Status.SUCCESS")
    assert(state_holder.status == py_trees.Status.SUCCESS)

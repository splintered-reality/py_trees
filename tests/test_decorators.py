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

from nose.tools import assert_raises
import time

##############################################################################
# Logging Level
##############################################################################

py_trees.logging.level = py_trees.logging.Level.DEBUG
logger = py_trees.logging.Logger("Nosetest")


##############################################################################
# Classes
##############################################################################

class InvalidSetup(py_trees.behaviour.Behaviour):
    def setup(self, timeout):
        # A common mistake is to forget to return a boolean value
        # Composite behaviours will at least check to make sure that
        # their children do so and raise TypeError's if they fail
        # to do so.
        pass


class DummyDecorator(py_trees.decorators.Decorator):
    def __init__(self, child, name=py_trees.common.Name.AUTO_GENERATED):
        super(DummyDecorator, self).__init__(name=name, child=child)

##############################################################################
# Tests
##############################################################################


def test_set_name():
    console.banner("Set Name")
    child = py_trees.behaviours.Success(name="Woohoo")
    named_decorator = DummyDecorator(name="Foo", child=child)
    no_named_decorator = DummyDecorator(child=child)
    print("\n--------- Assertions ---------\n")
    print("named_decorator.name == Foo")
    assert(named_decorator.name == "Foo")
    print("no_named_decorator.name == DummyDecorator\\n[Woohoo]")
    assert(no_named_decorator.name == "DummyDecorator\n[Woohoo]")


def test_invalid_child():
    console.banner("Invalid Child")
    print("\n--------- Assertions ---------\n")
    print("TypeError is raised")
    assert_raises(TypeError, DummyDecorator.__init__, child=5)


def test_invalid_setup():
    console.banner("Invalid Setup")
    parent = py_trees.decorators.Decorator(
        name="Decorator",
        child=InvalidSetup(name="Invalid Setup")
    )
    print("\n--------- Assertions ---------\n")
    print("TypeError is raised")
    with assert_raises(TypeError) as context:
        parent.setup(timeout=15)
    print("TypeError has message with substring 'NoneType'")
    assert("NoneType" in str(context.exception))


def test_failure_is_success_tree():
    console.banner("Failure is Success Tree")
    root = py_trees.composites.Selector(name="Root")
    failure = py_trees.behaviours.Failure(name="Failure")
    failure_is_success = py_trees.decorators.FailureIsSuccess(
        child=py_trees.behaviours.Failure()
    )
    root.add_child(failure)
    root.add_child(failure_is_success)
    py_trees.display.print_ascii_tree(root)
    visitor = py_trees.visitors.DebugVisitor()
    py_trees.tests.tick_tree(root, 1, 1, visitors=[visitor], print_snapshot=True)

    print("\n--------- Assertions ---------\n")
    print("root.status == py_trees.common.Status.SUCCESS")
    assert(root.status == py_trees.common.Status.SUCCESS)
    print("failure.status == py_trees.common.Status.FAILURE")
    assert(failure.status == py_trees.common.Status.FAILURE)
    print("failure_is_success.status == py_trees.common.Status.SUCCESS")
    assert(failure_is_success.status == py_trees.common.Status.SUCCESS)
    print("failure_is_success.decorated.status == py_trees.common.Status.FAILURE")
    assert(failure_is_success.decorated.status == py_trees.common.Status.FAILURE)


def test_failure_is_running_tree():
    console.banner("Failure is Running Tree")
    root = py_trees.composites.Parallel(name="Root")
    running = py_trees.decorators.FailureIsRunning(
        child=py_trees.behaviours.Running()
    )
    failure = py_trees.decorators.FailureIsRunning(
        child=py_trees.behaviours.Failure()
    )
    success = py_trees.decorators.FailureIsRunning(
        child=py_trees.behaviours.Success()
    )
    root.add_child(running)
    root.add_child(failure)
    root.add_child(success)
    visitor = py_trees.visitors.DebugVisitor()
    py_trees.tests.tick_tree(root, 1, 1, visitors=[visitor], print_snapshot=True)

    print("\n--------- Assertions ---------\n")
    print("running.status == py_trees.common.Status.RUNNING")
    assert(running.status == py_trees.common.Status.RUNNING)
    print("running.decorated.status == py_trees.common.Status.RUNNING")
    assert(running.decorated.status == py_trees.common.Status.RUNNING)
    print("failure.status == py_trees.common.Status.RUNNING")
    assert(failure.status == py_trees.common.Status.RUNNING)
    print("failure.decorated.status == py_trees.common.Status.FAILURE")
    assert(failure.decorated.status == py_trees.common.Status.FAILURE)
    print("success.status == py_trees.common.Status.SUCCESS")
    assert(success.status == py_trees.common.Status.SUCCESS)
    print("success.decorated.status == py_trees.common.Status.SUCCESS")
    assert(success.decorated.status == py_trees.common.Status.SUCCESS)
    print("root.status == py_trees.common.Status.RUNNING")
    assert(root.status == py_trees.common.Status.RUNNING)


def test_running_is_success_tree():
    console.banner("Running is Success Tree")
    root = py_trees.composites.Parallel(name="Root")
    running = py_trees.decorators.RunningIsSuccess(
        child=py_trees.behaviours.Running()
    )
    failure = py_trees.decorators.RunningIsSuccess(
        child=py_trees.behaviours.Failure()
    )
    success = py_trees.decorators.RunningIsSuccess(
        child=py_trees.behaviours.Success()
    )
    root.add_child(running)
    root.add_child(failure)
    root.add_child(success)
    visitor = py_trees.visitors.DebugVisitor()
    py_trees.tests.tick_tree(root, 1, 1, visitors=[visitor], print_snapshot=True)

    print("\n--------- Assertions ---------\n")
    print("running.status == py_trees.common.Status.SUCCESS")
    assert(running.status == py_trees.common.Status.SUCCESS)
    print("running.decorated.status == py_trees.common.Status.INVALID")
    assert(running.decorated.status == py_trees.common.Status.INVALID)
    print("failure.status == py_trees.common.Status.FAILURE")
    assert(failure.status == py_trees.common.Status.FAILURE)
    print("failure.decorated.status == py_trees.common.Status.FAILURE")
    assert(failure.decorated.status == py_trees.common.Status.FAILURE)
    print("success.status == py_trees.common.Status.SUCCESS")
    assert(success.status == py_trees.common.Status.SUCCESS)
    print("success.decorated.status == py_trees.common.Status.SUCCESS")
    assert(success.decorated.status == py_trees.common.Status.SUCCESS)
    print("root.status == py_trees.common.Status.FAILURE")
    assert(root.status == py_trees.common.Status.FAILURE)


def test_success_is_running_tree():
    console.banner("Success is Running Tree")
    root = py_trees.composites.Selector(name="Root")
    failure = py_trees.decorators.SuccessIsRunning(
        child=py_trees.behaviours.Failure()
    )
    parallel = py_trees.composites.Parallel(name="Parallel")
    running = py_trees.decorators.SuccessIsRunning(
        child=py_trees.behaviours.Running()
    )
    success = py_trees.decorators.SuccessIsRunning(
        child=py_trees.behaviours.Success()
    )
    root.add_child(failure)
    parallel.add_child(success)
    parallel.add_child(running)
    root.add_child(parallel)
    visitor = py_trees.visitors.DebugVisitor()
    py_trees.tests.tick_tree(root, 1, 1, visitors=[visitor], print_snapshot=True)

    print("\n--------- Assertions ---------\n")
    print("failure.status == py_trees.common.Status.FAILURE")
    assert(failure.status == py_trees.common.Status.FAILURE)
    print("failure.decorated.status == py_trees.common.Status.FAILURE")
    assert(failure.decorated.status == py_trees.common.Status.FAILURE)
    print("success.status == py_trees.common.Status.RUNNING")
    assert(success.status == py_trees.common.Status.RUNNING)
    print("success.decorated.status == py_trees.common.Status.SUCCESS")
    assert(success.decorated.status == py_trees.common.Status.SUCCESS)
    print("running.status == py_trees.common.Status.RUNNING")
    assert(running.status == py_trees.common.Status.RUNNING)
    print("running.decorated.status == py_trees.common.Status.RUNNING")
    assert(running.decorated.status == py_trees.common.Status.RUNNING)
    print("parallel.status == py_trees.common.Status.RUNNING")
    assert(parallel.status == py_trees.common.Status.RUNNING)
    print("root.status == py_trees.common.Status.RUNNING")
    assert(root.status == py_trees.common.Status.RUNNING)


def test_success_is_failure_tree():
    console.banner("Success is Failure Tree")
    root = py_trees.composites.Selector("Root")
    failure = py_trees.behaviours.Failure(name="Failure")
    success_is_failure = py_trees.decorators.SuccessIsFailure(
        name="Success Is Failure",
        child=py_trees.behaviours.Success()
    )
    root.add_child(failure)
    root.add_child(success_is_failure)
    py_trees.display.print_ascii_tree(root)
    visitor = py_trees.visitors.DebugVisitor()
    py_trees.tests.tick_tree(root, 1, 1, visitors=[visitor], print_snapshot=True)

    print("\n--------- Assertions ---------\n")
    print("failure.status == py_trees.common.Status.FAILURE")
    assert(failure.status == py_trees.common.Status.FAILURE)
    print("success_is_failure.status == py_trees.common.Status.FAILURE")
    assert(success_is_failure.status == py_trees.common.Status.FAILURE)
    print("success_is_failure.decorated.status == py_trees.common.Status.SUCCESS")
    assert(success_is_failure.decorated.status == py_trees.common.Status.SUCCESS)
    print("root.status == py_trees.common.Status.FAILURE")
    assert(root.status == py_trees.common.Status.FAILURE)


def test_inverter():
    console.banner("Inverter")
    root = py_trees.composites.Sequence(name="Root")
    selector = py_trees.composites.Selector(name="Selector")
    failure = py_trees.behaviours.Failure(name="Failure")
    success_inverter = py_trees.decorators.Inverter(child=py_trees.behaviours.Success())
    success = py_trees.behaviours.Success(name="Success")
    failure_inverter = py_trees.decorators.Inverter(py_trees.behaviours.Failure())
    selector.add_child(failure)
    selector.add_child(success_inverter)
    selector.add_child(success)
    root.add_child(selector)
    root.add_child(failure_inverter)
    py_trees.display.print_ascii_tree(root)
    visitor = py_trees.visitors.DebugVisitor()

    for i in range(0, 2):

        py_trees.tests.tick_tree(root, i, i, visitors=[visitor], print_snapshot=True)

        print("\n--------- Assertions ---------\n")
        print("success.status == py_trees.common.Status.SUCCESS")
        assert(success.status == py_trees.common.Status.SUCCESS)
        print("failure_inverter.status == py_trees.common.Status.SUCCESS")
        assert(failure_inverter.status == py_trees.common.Status.SUCCESS)
        print("failure_inverter.decorated.status == py_trees.common.Status.FAILURE")
        assert(failure_inverter.decorated.status == py_trees.common.Status.FAILURE)
        print("root.status == py_trees.common.Status.SUCCESS")
        assert(root.status == py_trees.common.Status.SUCCESS)
        print("failure.status == py_trees.common.Status.FAILURE")
        assert(failure.status == py_trees.common.Status.FAILURE)
        print("success_inverter.status == py_trees.common.Status.FAILURE")
        assert(success_inverter.status == py_trees.common.Status.FAILURE)
        print("success_inverter.decorated.status == py_trees.common.Status.SUCCESS")
        assert(success_inverter.decorated.status == py_trees.common.Status.SUCCESS)


def test_running_is_failure_tree():
    console.banner("Running is Failure Tree")
    root = py_trees.composites.Selector(name="Root")
    running = py_trees.decorators.RunningIsFailure(
        child=py_trees.behaviours.Running()
    )
    failure = py_trees.decorators.RunningIsFailure(
        child=py_trees.behaviours.Failure()
    )
    success = py_trees.decorators.RunningIsFailure(
        child=py_trees.behaviours.Success()
    )
    root.add_child(running)
    root.add_child(failure)
    root.add_child(success)
    visitor = py_trees.visitors.DebugVisitor()
    py_trees.tests.tick_tree(root, 1, 1, visitors=[visitor], print_snapshot=True)

    print("\n--------- Assertions ---------\n")
    print("running.status == py_trees.common.Status.FAILURE")
    assert(running.status == py_trees.common.Status.FAILURE)
    print("running.decorated.status == py_trees.common.Status.INVALID")
    assert(running.decorated.status == py_trees.common.Status.INVALID)
    print("failure.status == py_trees.common.Status.FAILURE")
    assert(failure.status == py_trees.common.Status.FAILURE)
    print("failure.decorated.status == py_trees.common.Status.FAILURE")
    assert(failure.decorated.status == py_trees.common.Status.FAILURE)
    print("success.status == py_trees.common.Status.SUCCESS")
    assert(success.status == py_trees.common.Status.SUCCESS)
    print("success.decorated.status == py_trees.common.Status.SUCCESS")
    assert(success.decorated.status == py_trees.common.Status.SUCCESS)
    print("root.status == py_trees.common.Status.SUCCESS")
    assert(root.status == py_trees.common.Status.SUCCESS)


def test_timeout():
    console.banner("Timeout")
    running = py_trees.behaviours.Running(name="Running")
    timeout = py_trees.decorators.Timeout(child=running, duration=0.2)
    py_trees.display.print_ascii_tree(timeout)
    visitor = py_trees.visitors.DebugVisitor()

    # Test that it times out and re-initialises properly
    for i in range(0, 2):
        py_trees.tests.tick_tree(timeout, 2*i+1, 2*i+1, visitors=[visitor])

        print("\n--------- Assertions ---------\n")
        print("timeout.status == py_trees.common.Status.RUNNING")
        assert(timeout.status == py_trees.common.Status.RUNNING)
        print("running.status == py_trees.common.Status.RUNNING")
        assert(running.status == py_trees.common.Status.RUNNING)

        time.sleep(0.3)
        py_trees.tests.tick_tree(timeout, 2*i+2, 2*i+2, visitors=[visitor])

        print("\n--------- Assertions ---------\n")
        print("timeout.status == py_trees.common.Status.FAILURE")
        assert(timeout.status == py_trees.common.Status.FAILURE)
        print("running.status == py_trees.common.Status.INVALID")
        assert(running.status == py_trees.common.Status.INVALID)

    # test that it passes on success
    count = py_trees.behaviours.Count(
        name="Count",
        fail_until=0,
        running_until=1,
        success_until=10,
        reset=False
    )
    timeout = py_trees.decorators.Timeout(child=count, duration=0.2)
    py_trees.display.print_ascii_tree(timeout)

    py_trees.tests.tick_tree(timeout, 1, 1, visitors=[visitor])

    print("\n--------- Assertions ---------\n")
    print("timeout.status == py_trees.common.Status.RUNNING")
    assert(timeout.status == py_trees.common.Status.RUNNING)
    print("count.status == py_trees.common.Status.RUNNING")
    assert(count.status == py_trees.common.Status.RUNNING)

    py_trees.tests.tick_tree(timeout, 2, 2, visitors=[visitor])

    print("\n--------- Assertions ---------\n")
    print("timeout.status == py_trees.common.Status.SUCCESS")
    assert(timeout.status == py_trees.common.Status.SUCCESS)
    print("count.status == py_trees.common.Status.SUCCESS")
    assert(count.status == py_trees.common.Status.SUCCESS)

    # test that it passes on failure
    failure = py_trees.behaviours.Failure()
    timeout = py_trees.decorators.Timeout(child=failure, duration=0.2)
    py_trees.display.print_ascii_tree(timeout)

    py_trees.tests.tick_tree(timeout, 1, 1, visitors=[visitor])

    print("\n--------- Assertions ---------\n")
    print("timeout.status == py_trees.common.Status.FAILURE")
    assert(timeout.status == py_trees.common.Status.FAILURE)
    print("failure.status == py_trees.common.Status.FAILURE")
    assert(failure.status == py_trees.common.Status.FAILURE)


def test_condition():
    console.banner("Condition")

    child = py_trees.behaviours.Count(
        name="Count",
        fail_until=2,
        running_until=2,
        success_until=10,
        reset=True
    )
    condition = py_trees.decorators.Condition(
        child=child,
        status=py_trees.common.Status.SUCCESS
    )

    visitor = py_trees.visitors.DebugVisitor()
    py_trees.tests.tick_tree(condition, 1, 1, visitors=[visitor], print_snapshot=True)

    print("\n--------- Assertions ---------\n")
    print("child.status == py_trees.common.Status.FAILURE")
    assert(child.status == py_trees.common.Status.FAILURE)
    print("condition.status == py_trees.common.Status.RUNNING")
    assert(condition.status == py_trees.common.Status.RUNNING)

    py_trees.tests.tick_tree(condition, 2, 2, visitors=[visitor], print_snapshot=True)

    print("\n--------- Assertions ---------\n")
    print("child.status == py_trees.common.Status.FAILURE")
    assert(child.status == py_trees.common.Status.FAILURE)
    print("condition.status == py_trees.common.Status.RUNNING")
    assert(condition.status == py_trees.common.Status.RUNNING)

    py_trees.tests.tick_tree(condition, 3, 3, visitors=[visitor], print_snapshot=True)

    print("\n--------- Assertions ---------\n")
    print("child.status == py_trees.common.Status.SUCCESS")
    assert(child.status == py_trees.common.Status.SUCCESS)
    print("condition.status == py_trees.common.Status.SUCCESS")
    assert(condition.status == py_trees.common.Status.SUCCESS)

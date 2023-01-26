#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/splintered-reality/py_trees/devel/LICENSE
#

##############################################################################
# Imports
##############################################################################

import time

import py_trees
import py_trees.console as console
import py_trees.tests
import pytest

##############################################################################
# Logging Level
##############################################################################

py_trees.logging.level = py_trees.logging.Level.DEBUG
logger = py_trees.logging.Logger("Tests")


##############################################################################
# Classes
##############################################################################


class DummyDecorator(py_trees.decorators.Decorator):
    def __init__(self, name: str, child: py_trees.behaviour.Behaviour):
        super(DummyDecorator, self).__init__(name=name, child=child)

    def update(self) -> py_trees.common.Status:
        return py_trees.common.Status.INVALID


##############################################################################
# Tests
##############################################################################


def test_invalid_child() -> None:
    console.banner("Decorator Exceptions - Invalid Child Type")
    with pytest.raises(TypeError) as context:  # if raised, context survives
        # intentional error -> silence mypy
        decorator = DummyDecorator(name="dummy", child=5)  # type: ignore[arg-type]
        print(f"Name: {decorator.name}")
        py_trees.tests.print_assert_details("TypeError raised", "raised", "not raised")
    py_trees.tests.print_assert_details("TypeError raised", "yes", "yes")
    assert "TypeError" == context.typename
    py_trees.tests.print_assert_details(
        "Substring match", "instance", f"{context.value}"
    )
    assert "instance" in str(context.value)


def test_repeat_until_success() -> None:
    console.banner("Repeat - Until Success")
    child = py_trees.behaviours.StatusQueue(
        name="R-S-S",
        queue=[
            py_trees.common.Status.RUNNING,
            py_trees.common.Status.SUCCESS,
            py_trees.common.Status.SUCCESS,
        ],
        eventually=None,
    )
    decorator = py_trees.decorators.Repeat(name="Repeat", num_success=2, child=child)
    decorator.tick_once()
    print("\n--------- Tick 1 ---------\n")
    print("decorator.status == py_trees.common.Status.RUNNING")
    assert decorator.status == py_trees.common.Status.RUNNING
    print("child.status == py_trees.common.Status.RUNNING")
    assert child.status == py_trees.common.Status.RUNNING

    decorator.tick_once()
    print("\n--------- Tick 2 ---------\n")
    print("decorator.status == py_trees.common.Status.RUNNING")
    assert decorator.status == py_trees.common.Status.RUNNING
    print("child.status == py_trees.common.Status.SUCCESS")
    assert child.status == py_trees.common.Status.SUCCESS

    decorator.tick_once()
    print("\n--------- Tick 3 ---------\n")
    print("decorator.status == py_trees.common.Status.SUCCESS")
    assert decorator.status == py_trees.common.Status.SUCCESS
    print("child.status == py_trees.common.Status.SUCCESS")
    assert child.status == py_trees.common.Status.SUCCESS


def test_repeat_interrupting_failure() -> None:
    console.banner("Repeat - Interrupting Failure")
    child = py_trees.behaviours.StatusQueue(
        name="R-S-F",
        queue=[
            py_trees.common.Status.RUNNING,
            py_trees.common.Status.SUCCESS,
            py_trees.common.Status.FAILURE,
        ],
        eventually=None,
    )
    decorator = py_trees.decorators.Repeat(name="Repeat", num_success=2, child=child)
    decorator.tick_once()
    print("\n--------- Tick 1 ---------\n")
    print("decorator.status == py_trees.common.Status.RUNNING")
    assert decorator.status == py_trees.common.Status.RUNNING
    print("child.status == py_trees.common.Status.RUNNING")
    assert child.status == py_trees.common.Status.RUNNING

    decorator.tick_once()
    print("\n--------- Tick 2 ---------\n")
    print("decorator.status == py_trees.common.Status.RUNNING")
    assert decorator.status == py_trees.common.Status.RUNNING
    print("child.status == py_trees.common.Status.SUCCESS")
    assert child.status == py_trees.common.Status.SUCCESS

    decorator.tick_once()
    print("\n--------- Tick 3 ---------\n")
    print("decorator.status == py_trees.common.Status.FAILURE")
    assert decorator.status == py_trees.common.Status.FAILURE
    print("child.status == py_trees.common.Status.FAILURE")
    assert child.status == py_trees.common.Status.FAILURE


def test_retry_until_success() -> None:
    console.banner("Retry - Until Success")
    child = py_trees.behaviours.StatusQueue(
        name="R-F-S",
        queue=[
            py_trees.common.Status.RUNNING,
            py_trees.common.Status.FAILURE,
            py_trees.common.Status.SUCCESS,
        ],
        eventually=None,
    )
    decorator = py_trees.decorators.Retry(name="Retry", num_failures=2, child=child)
    decorator.tick_once()
    print("\n--------- Tick 1 ---------\n")
    print("decorator.status == py_trees.common.Status.RUNNING")
    assert decorator.status == py_trees.common.Status.RUNNING
    print("decorator.failures == 0")
    assert decorator.failures == 0
    print("child.status == py_trees.common.Status.RUNNING")
    assert child.status == py_trees.common.Status.RUNNING

    decorator.tick_once()
    print("\n--------- Tick 2 ---------\n")
    print("decorator.status == py_trees.common.Status.RUNNING")
    assert decorator.status == py_trees.common.Status.RUNNING
    print("decorator.failures == 1")
    assert decorator.failures == 1
    print("child.status == py_trees.common.Status.FAILURE")
    assert child.status == py_trees.common.Status.FAILURE

    decorator.tick_once()
    print("\n--------- Tick 3 ---------\n")
    print("decorator.status == py_trees.common.Status.SUCCESS")
    assert decorator.status == py_trees.common.Status.SUCCESS
    print("decorator.failures == 1")
    assert decorator.failures == 1
    print("child.status == py_trees.common.Status.SUCCESS")
    assert child.status == py_trees.common.Status.SUCCESS


def test_retry_eventual_failure() -> None:
    console.banner("Retry - Eventual Failure")
    child = py_trees.behaviours.StatusQueue(
        name="R-F-F",
        queue=[
            py_trees.common.Status.RUNNING,
            py_trees.common.Status.FAILURE,
            py_trees.common.Status.FAILURE,
        ],
        eventually=None,
    )
    decorator = py_trees.decorators.Retry(name="Retry", num_failures=2, child=child)
    decorator.tick_once()
    print("\n--------- Tick 1 ---------\n")
    print("decorator.status == py_trees.common.Status.RUNNING")
    assert decorator.status == py_trees.common.Status.RUNNING
    print("decorator.failures == 0")
    assert decorator.failures == 0
    print("child.status == py_trees.common.Status.RUNNING")
    assert child.status == py_trees.common.Status.RUNNING

    decorator.tick_once()
    print("\n--------- Tick 2 ---------\n")
    print("decorator.status == py_trees.common.Status.RUNNING")
    assert decorator.status == py_trees.common.Status.RUNNING
    print("decorator.failures == 1")
    assert decorator.failures == 1
    print("child.status == py_trees.common.Status.FAILURE")
    assert child.status == py_trees.common.Status.FAILURE

    decorator.tick_once()
    print("\n--------- Tick 3 ---------\n")
    print("decorator.status == py_trees.common.Status.FAILURE")
    assert decorator.status == py_trees.common.Status.FAILURE
    print("decorator.failures == 2")
    assert decorator.failures == 2
    print("child.status == py_trees.common.Status.FAILURE")
    assert child.status == py_trees.common.Status.FAILURE


def test_failure_is_success_tree() -> None:
    console.banner("Failure is Success Tree")
    root = py_trees.composites.Selector(name="Root", memory=False)
    failure = py_trees.behaviours.Failure(name="Failure")
    failure_is_success = py_trees.decorators.FailureIsSuccess(
        name="Failure Is Success", child=py_trees.behaviours.Failure(name="Failure")
    )
    root.add_child(failure)
    root.add_child(failure_is_success)
    print(py_trees.display.unicode_tree(root))
    visitor = py_trees.visitors.DebugVisitor()
    py_trees.tests.tick_tree(root, 1, 1, visitors=[visitor], print_snapshot=True)

    print("\n--------- Assertions ---------\n")
    print("root.status == py_trees.common.Status.SUCCESS")
    assert root.status == py_trees.common.Status.SUCCESS
    print("failure.status == py_trees.common.Status.FAILURE")
    assert failure.status == py_trees.common.Status.FAILURE
    print("failure_is_success.status == py_trees.common.Status.SUCCESS")
    assert failure_is_success.status == py_trees.common.Status.SUCCESS
    print("failure_is_success.decorated.status == py_trees.common.Status.FAILURE")
    assert failure_is_success.decorated.status == py_trees.common.Status.FAILURE


def test_failure_is_running_tree() -> None:
    console.banner("Failure is Running Tree")
    root = py_trees.composites.Parallel(
        name="Root", policy=py_trees.common.ParallelPolicy.SuccessOnAll()
    )
    running = py_trees.decorators.FailureIsRunning(
        name="Failure Is Running", child=py_trees.behaviours.Running(name="Running")
    )
    failure = py_trees.decorators.FailureIsRunning(
        name="Failure Is Running", child=py_trees.behaviours.Failure(name="Failure")
    )
    success = py_trees.decorators.FailureIsRunning(
        name="Failure Is Running", child=py_trees.behaviours.Success(name="Success")
    )
    root.add_child(running)
    root.add_child(failure)
    root.add_child(success)
    visitor = py_trees.visitors.DebugVisitor()
    py_trees.tests.tick_tree(root, 1, 1, visitors=[visitor], print_snapshot=True)

    print("\n--------- Assertions ---------\n")
    print("running.status == py_trees.common.Status.RUNNING")
    assert running.status == py_trees.common.Status.RUNNING
    print("running.decorated.status == py_trees.common.Status.RUNNING")
    assert running.decorated.status == py_trees.common.Status.RUNNING
    print("failure.status == py_trees.common.Status.RUNNING")
    assert failure.status == py_trees.common.Status.RUNNING
    print("failure.decorated.status == py_trees.common.Status.FAILURE")
    assert failure.decorated.status == py_trees.common.Status.FAILURE
    print("success.status == py_trees.common.Status.SUCCESS")
    assert success.status == py_trees.common.Status.SUCCESS
    print("success.decorated.status == py_trees.common.Status.SUCCESS")
    assert success.decorated.status == py_trees.common.Status.SUCCESS
    print("root.status == py_trees.common.Status.RUNNING")
    assert root.status == py_trees.common.Status.RUNNING


def test_running_is_success_tree() -> None:
    console.banner("Running is Success Tree")
    root = py_trees.composites.Parallel(
        name="Root", policy=py_trees.common.ParallelPolicy.SuccessOnAll()
    )
    running = py_trees.decorators.RunningIsSuccess(
        name="Running is Success", child=py_trees.behaviours.Running(name="Running")
    )
    failure = py_trees.decorators.RunningIsSuccess(
        name="Running is Success", child=py_trees.behaviours.Failure(name="Failure")
    )
    success = py_trees.decorators.RunningIsSuccess(
        name="Running is Success", child=py_trees.behaviours.Success(name="Success")
    )
    root.add_child(running)
    root.add_child(failure)
    root.add_child(success)
    visitor = py_trees.visitors.DebugVisitor()
    py_trees.tests.tick_tree(root, 1, 1, visitors=[visitor], print_snapshot=True)

    print("\n--------- Assertions ---------\n")
    print("running.status == py_trees.common.Status.SUCCESS")
    assert running.status == py_trees.common.Status.SUCCESS
    print("running.decorated.status == py_trees.common.Status.INVALID")
    assert running.decorated.status == py_trees.common.Status.INVALID
    print("failure.status == py_trees.common.Status.FAILURE")
    assert failure.status == py_trees.common.Status.FAILURE
    print("failure.decorated.status == py_trees.common.Status.FAILURE")
    assert failure.decorated.status == py_trees.common.Status.FAILURE
    print("success.status == py_trees.common.Status.SUCCESS")
    assert success.status == py_trees.common.Status.SUCCESS
    print("success.decorated.status == py_trees.common.Status.SUCCESS")
    assert success.decorated.status == py_trees.common.Status.SUCCESS
    print("root.status == py_trees.common.Status.FAILURE")
    assert root.status == py_trees.common.Status.FAILURE


def test_success_is_running_tree() -> None:
    console.banner("Success is Running Tree")
    root = py_trees.composites.Selector(name="Root", memory=False)
    failure = py_trees.decorators.SuccessIsRunning(
        name="Success is Running", child=py_trees.behaviours.Failure(name="Failure")
    )
    parallel = py_trees.composites.Parallel(
        name="Parallel", policy=py_trees.common.ParallelPolicy.SuccessOnAll()
    )
    running = py_trees.decorators.SuccessIsRunning(
        name="Success is Running", child=py_trees.behaviours.Running(name="Running")
    )
    success = py_trees.decorators.SuccessIsRunning(
        name="Success is Running", child=py_trees.behaviours.Success(name="Success")
    )
    root.add_child(failure)
    parallel.add_child(success)
    parallel.add_child(running)
    root.add_child(parallel)
    visitor = py_trees.visitors.DebugVisitor()
    py_trees.tests.tick_tree(root, 1, 1, visitors=[visitor], print_snapshot=True)

    print("\n--------- Assertions ---------\n")
    print("failure.status == py_trees.common.Status.FAILURE")
    assert failure.status == py_trees.common.Status.FAILURE
    print("failure.decorated.status == py_trees.common.Status.FAILURE")
    assert failure.decorated.status == py_trees.common.Status.FAILURE
    print("success.status == py_trees.common.Status.RUNNING")
    assert success.status == py_trees.common.Status.RUNNING
    print("success.decorated.status == py_trees.common.Status.SUCCESS")
    assert success.decorated.status == py_trees.common.Status.SUCCESS
    print("running.status == py_trees.common.Status.RUNNING")
    assert running.status == py_trees.common.Status.RUNNING
    print("running.decorated.status == py_trees.common.Status.RUNNING")
    assert running.decorated.status == py_trees.common.Status.RUNNING
    print("parallel.status == py_trees.common.Status.RUNNING")
    assert parallel.status == py_trees.common.Status.RUNNING
    print("root.status == py_trees.common.Status.RUNNING")
    assert root.status == py_trees.common.Status.RUNNING


def test_success_is_failure_tree() -> None:
    console.banner("Success is Failure Tree")
    root = py_trees.composites.Selector(name="Root", memory=False)
    failure = py_trees.behaviours.Failure(name="Failure")
    success_is_failure = py_trees.decorators.SuccessIsFailure(
        name="Success Is Failure", child=py_trees.behaviours.Success(name="Success")
    )
    root.add_child(failure)
    root.add_child(success_is_failure)
    print(py_trees.display.unicode_tree(root))
    visitor = py_trees.visitors.DebugVisitor()
    py_trees.tests.tick_tree(root, 1, 1, visitors=[visitor], print_snapshot=True)

    print("\n--------- Assertions ---------\n")
    print("failure.status == py_trees.common.Status.FAILURE")
    assert failure.status == py_trees.common.Status.FAILURE
    print("success_is_failure.status == py_trees.common.Status.FAILURE")
    assert success_is_failure.status == py_trees.common.Status.FAILURE
    print("success_is_failure.decorated.status == py_trees.common.Status.SUCCESS")
    assert success_is_failure.decorated.status == py_trees.common.Status.SUCCESS
    print("root.status == py_trees.common.Status.FAILURE")
    assert root.status == py_trees.common.Status.FAILURE


def test_inverter() -> None:
    console.banner("Inverter")
    root = py_trees.composites.Sequence(name="Root", memory=True)
    selector = py_trees.composites.Selector(name="Selector", memory=False)
    failure = py_trees.behaviours.Failure(name="Failure")
    success_inverter = py_trees.decorators.Inverter(
        name="Inverter", child=py_trees.behaviours.Success(name="Success")
    )
    success = py_trees.behaviours.Success(name="Success")
    failure_inverter = py_trees.decorators.Inverter(
        name="Inverter", child=py_trees.behaviours.Failure(name="Failure")
    )
    selector.add_child(failure)
    selector.add_child(success_inverter)
    selector.add_child(success)
    root.add_child(selector)
    root.add_child(failure_inverter)
    print(py_trees.display.unicode_tree(root))
    visitor = py_trees.visitors.DebugVisitor()

    for i in range(0, 2):
        py_trees.tests.tick_tree(root, i, i, visitors=[visitor], print_snapshot=True)

        print("\n--------- Assertions ---------\n")
        print("success.status == py_trees.common.Status.SUCCESS")
        assert success.status == py_trees.common.Status.SUCCESS
        print("failure_inverter.status == py_trees.common.Status.SUCCESS")
        assert failure_inverter.status == py_trees.common.Status.SUCCESS
        print("failure_inverter.decorated.status == py_trees.common.Status.FAILURE")
        assert failure_inverter.decorated.status == py_trees.common.Status.FAILURE
        print("root.status == py_trees.common.Status.SUCCESS")
        assert root.status == py_trees.common.Status.SUCCESS
        print("failure.status == py_trees.common.Status.FAILURE")
        assert failure.status == py_trees.common.Status.FAILURE
        print("success_inverter.status == py_trees.common.Status.FAILURE")
        assert success_inverter.status == py_trees.common.Status.FAILURE
        print("success_inverter.decorated.status == py_trees.common.Status.SUCCESS")
        assert success_inverter.decorated.status == py_trees.common.Status.SUCCESS


def test_running_is_failure_tree() -> None:
    console.banner("Running is Failure Tree")
    root = py_trees.composites.Selector(name="Root", memory=False)
    running = py_trees.decorators.RunningIsFailure(
        name="Running is Failure", child=py_trees.behaviours.Running(name="Running")
    )
    failure = py_trees.decorators.RunningIsFailure(
        name="Running is Failure", child=py_trees.behaviours.Failure(name="Failure")
    )
    success = py_trees.decorators.RunningIsFailure(
        name="Running is Failure", child=py_trees.behaviours.Success(name="Success")
    )
    root.add_child(running)
    root.add_child(failure)
    root.add_child(success)
    visitor = py_trees.visitors.DebugVisitor()
    py_trees.tests.tick_tree(root, 1, 1, visitors=[visitor], print_snapshot=True)

    print("\n--------- Assertions ---------\n")
    print("running.status == py_trees.common.Status.FAILURE")
    assert running.status == py_trees.common.Status.FAILURE
    print("running.decorated.status == py_trees.common.Status.INVALID")
    assert running.decorated.status == py_trees.common.Status.INVALID
    print("failure.status == py_trees.common.Status.FAILURE")
    assert failure.status == py_trees.common.Status.FAILURE
    print("failure.decorated.status == py_trees.common.Status.FAILURE")
    assert failure.decorated.status == py_trees.common.Status.FAILURE
    print("success.status == py_trees.common.Status.SUCCESS")
    assert success.status == py_trees.common.Status.SUCCESS
    print("success.decorated.status == py_trees.common.Status.SUCCESS")
    assert success.decorated.status == py_trees.common.Status.SUCCESS
    print("root.status == py_trees.common.Status.SUCCESS")
    assert root.status == py_trees.common.Status.SUCCESS


def test_timeout() -> None:
    console.banner("Timeout")
    running = py_trees.behaviours.Running(name="Running")
    timeout = py_trees.decorators.Timeout(name="Timeout", child=running, duration=0.2)
    print(py_trees.display.unicode_tree(timeout))
    visitor = py_trees.visitors.DebugVisitor()

    # Test that it times out and re-initialises properly
    for i in range(0, 2):
        py_trees.tests.tick_tree(timeout, 2 * i + 1, 2 * i + 1, visitors=[visitor])

        print("\n--------- Assertions ---------\n")
        print("timeout.status == py_trees.common.Status.RUNNING")
        assert timeout.status == py_trees.common.Status.RUNNING
        print("running.status == py_trees.common.Status.RUNNING")
        assert running.status == py_trees.common.Status.RUNNING

        time.sleep(0.3)
        py_trees.tests.tick_tree(timeout, 2 * i + 2, 2 * i + 2, visitors=[visitor])

        print("\n--------- Assertions ---------\n")
        print("timeout.status == py_trees.common.Status.FAILURE")
        assert timeout.status == py_trees.common.Status.FAILURE
        print("running.status == py_trees.common.Status.INVALID")
        assert running.status == py_trees.common.Status.INVALID

    # test that it passes on success
    count = py_trees.behaviours.StatusQueue(
        name="Queue",
        queue=[py_trees.common.Status.RUNNING],
        eventually=py_trees.common.Status.SUCCESS,
    )
    timeout = py_trees.decorators.Timeout(name="Timeout", child=count, duration=0.2)
    print(py_trees.display.unicode_tree(timeout))

    py_trees.tests.tick_tree(timeout, 1, 1, visitors=[visitor])

    print("\n--------- Assertions ---------\n")
    print("timeout.status == py_trees.common.Status.RUNNING")
    assert timeout.status == py_trees.common.Status.RUNNING
    print("count.status == py_trees.common.Status.RUNNING")
    assert count.status == py_trees.common.Status.RUNNING

    py_trees.tests.tick_tree(timeout, 2, 2, visitors=[visitor])

    print("\n--------- Assertions ---------\n")
    print("timeout.status == py_trees.common.Status.SUCCESS")
    assert timeout.status == py_trees.common.Status.SUCCESS
    print("count.status == py_trees.common.Status.SUCCESS")
    assert count.status == py_trees.common.Status.SUCCESS

    # test that it passes on failure
    failure = py_trees.behaviours.Failure(name="Failure")
    timeout = py_trees.decorators.Timeout(name="Timeout", child=failure, duration=0.2)
    print(py_trees.display.unicode_tree(timeout))

    py_trees.tests.tick_tree(timeout, 1, 1, visitors=[visitor])

    print("\n--------- Assertions ---------\n")
    print("timeout.status == py_trees.common.Status.FAILURE")
    assert timeout.status == py_trees.common.Status.FAILURE
    print("failure.status == py_trees.common.Status.FAILURE")
    assert failure.status == py_trees.common.Status.FAILURE

    # test that it succeeds if child succeeds on last tick
    count = py_trees.behaviours.StatusQueue(
        name="Queue",
        queue=[py_trees.common.Status.RUNNING],
        eventually=py_trees.common.Status.SUCCESS,
    )
    timeout = py_trees.decorators.Timeout(name="Timeout", child=count, duration=0.1)
    print(py_trees.display.unicode_tree(timeout))

    py_trees.tests.tick_tree(timeout, 1, 1, visitors=[visitor])

    print("\n--------- Assertions ---------\n")
    print("timeout.status == py_trees.common.Status.RUNNING")
    assert timeout.status == py_trees.common.Status.RUNNING
    print("count.status == py_trees.common.Status.RUNNING")
    assert count.status == py_trees.common.Status.RUNNING

    time.sleep(0.2)  # go past the duration
    py_trees.tests.tick_tree(timeout, 2, 2, visitors=[visitor])

    print("\n--------- Assertions ---------\n")
    print("timeout.status == py_trees.common.Status.SUCCESS")
    assert timeout.status == py_trees.common.Status.SUCCESS
    print("count.status == py_trees.common.Status.SUCCESS")
    assert count.status == py_trees.common.Status.SUCCESS


def test_condition() -> None:
    console.banner("Condition")

    child = py_trees.behaviours.StatusQueue(
        name="Queue",
        queue=[
            py_trees.common.Status.FAILURE,
            py_trees.common.Status.FAILURE,
        ],
        eventually=py_trees.common.Status.SUCCESS,
    )
    condition = py_trees.decorators.Condition(
        name="Condition", child=child, status=py_trees.common.Status.SUCCESS
    )

    visitor = py_trees.visitors.DebugVisitor()
    py_trees.tests.tick_tree(condition, 1, 1, visitors=[visitor], print_snapshot=True)

    print("\n--------- Assertions ---------\n")
    print("child.status == py_trees.common.Status.FAILURE")
    assert child.status == py_trees.common.Status.FAILURE
    print("condition.status == py_trees.common.Status.RUNNING")
    assert condition.status == py_trees.common.Status.RUNNING

    py_trees.tests.tick_tree(condition, 2, 2, visitors=[visitor], print_snapshot=True)

    print("\n--------- Assertions ---------\n")
    print("child.status == py_trees.common.Status.FAILURE")
    assert child.status == py_trees.common.Status.FAILURE
    print("condition.status == py_trees.common.Status.RUNNING")
    assert condition.status == py_trees.common.Status.RUNNING

    py_trees.tests.tick_tree(condition, 3, 3, visitors=[visitor], print_snapshot=True)

    print("\n--------- Assertions ---------\n")
    print("child.status == py_trees.common.Status.SUCCESS")
    assert child.status == py_trees.common.Status.SUCCESS
    print("condition.status == py_trees.common.Status.SUCCESS")
    assert condition.status == py_trees.common.Status.SUCCESS


def test_status_to_blackboard() -> None:
    console.banner("Status to Blackboard")

    child = py_trees.behaviours.Success(name="Success")
    decorator = py_trees.decorators.StatusToBlackboard(
        name="Status2BB", child=child, variable_name="foo"
    )
    blackboard = py_trees.blackboard.Client()
    blackboard.register_key(key="foo", access=py_trees.common.Access.READ)
    decorator.tick_once()

    py_trees.tests.print_assert_banner()
    py_trees.tests.print_assert_details(
        text="Blackboard Variable (foo)",
        expected=py_trees.common.Status.SUCCESS,
        result=blackboard.foo,
    )
    assert blackboard.foo == py_trees.common.Status.SUCCESS
    py_trees.tests.print_assert_details(
        text="Child Status",
        expected=py_trees.common.Status.SUCCESS,
        result=child.status,
    )
    assert child.status == py_trees.common.Status.SUCCESS
    py_trees.tests.print_assert_details(
        text="Decorator Status",
        expected=py_trees.common.Status.SUCCESS,
        result=decorator.status,
    )
    assert decorator.status == py_trees.common.Status.SUCCESS

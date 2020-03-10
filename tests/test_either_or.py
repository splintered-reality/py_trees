#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/splintered-reality/py_trees/devel/LICENSE
#
##############################################################################
# Imports
##############################################################################

import operator
import nose

import py_trees
import py_trees.console as console

##############################################################################
# Helpers
##############################################################################


def assert_banner():
    print(console.green + "----- Asserts -----" + console.reset)


def assert_details(text, expected, result):
    print(console.green + text +
          "." * (40 - len(text)) +
          console.cyan + "{}".format(expected) +
          console.yellow + " [{}]".format(result) +
          console.reset)


##############################################################################
# Either Or
##############################################################################


def create_root():
    trigger_one = py_trees.decorators.FailureIsRunning(
        name="FisR",
        child=py_trees.behaviours.SuccessEveryN(
            name="Joystick 1",
            n=4
        )
    )
    trigger_two = py_trees.decorators.FailureIsRunning(
        name="FisR",
        child=py_trees.behaviours.SuccessEveryN(
            name="Joystick 2",
            n=7
        )
    )
    enable_joystick_one = py_trees.behaviours.SetBlackboardVariable(
        name="Joy1 - Enabled",
        variable_name="joystick_one",
        variable_value="enabled")
    enable_joystick_two = py_trees.behaviours.SetBlackboardVariable(
        name="Joy2 - Enabled",
        variable_name="joystick_two",
        variable_value="enabled")
    reset_joystick_one = py_trees.behaviours.SetBlackboardVariable(
        name="Joy1 - Disabled",
        variable_name="joystick_one",
        variable_value="disabled")
    reset_joystick_two = py_trees.behaviours.SetBlackboardVariable(
        name="Joy2 - Disabled",
        variable_name="joystick_two",
        variable_value="disabled")
    task_one = py_trees.behaviours.TickCounter(
        name="Task 1",
        duration=2,
        completion_status=py_trees.common.Status.SUCCESS
    )
    task_two = py_trees.behaviours.TickCounter(
        name="Task 2",
        duration=2,
        completion_status=py_trees.common.Status.SUCCESS
    )
    idle = py_trees.behaviours.Running(name="Idle")
    either_or = py_trees.idioms.either_or(
        name="EitherOr",
        conditions=[
            py_trees.common.ComparisonExpression("joystick_one", "enabled", operator.eq),
            py_trees.common.ComparisonExpression("joystick_two", "enabled", operator.eq),
        ],
        subtrees=[task_one, task_two],
        namespace="either_or",
    )
    root = py_trees.composites.Parallel(
        name="Root",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=False)
    )
    reset = py_trees.composites.Sequence(name="Reset")
    reset.add_children([reset_joystick_one, reset_joystick_two])
    joystick_one_events = py_trees.composites.Sequence(name="Joy1 Events")
    joystick_one_events.add_children([trigger_one, enable_joystick_one])
    joystick_two_events = py_trees.composites.Sequence(name="Joy2 Events")
    joystick_two_events.add_children([trigger_two, enable_joystick_two])
    tasks = py_trees.composites.Selector(name="Tasks")
    tasks.add_children([either_or, idle])
    root.add_children([reset, joystick_one_events, joystick_two_events, tasks])
    return root, task_one, task_two


def test_basic_workflow():
    # same as py-trees-demo-idiom-either-or
    root, task_one, task_two = create_root()
    # tree = py_trees.trees.BehaviourTree(root=root)
    root.tick_once()
    # tree.tick()
    assert_banner()
    assert_details(
        text="Tick 1 - tasks not yet ticked",
        expected=py_trees.common.Status.INVALID,
        result=task_one.status,
    )
    assert(task_one.status == py_trees.common.Status.INVALID)
    root.tick_once()
    root.tick_once()
    root.tick_once()
    assert_details(
        text="Tick 4 - task one running",
        expected=py_trees.common.Status.RUNNING,
        result=task_one.status,
    )
    assert(task_one.status == py_trees.common.Status.RUNNING)
    root.tick_once()
    root.tick_once()
    assert_details(
        text="Tick 6 - task one finished",
        expected=py_trees.common.Status.SUCCESS,
        result=task_one.status,
    )
    assert(task_one.status == py_trees.common.Status.SUCCESS)
    root.tick_once()
    assert_details(
        text="Tick 7 - task two starts",
        expected=py_trees.common.Status.RUNNING,
        result=task_two.status,
    )
    assert(task_two.status == py_trees.common.Status.RUNNING)
    root.tick_once()
    assert_details(
        text="Tick 8 - task one ignored",
        expected=py_trees.common.Status.INVALID,
        result=task_one.status,
    )
    assert(task_one.status == py_trees.common.Status.INVALID)
    root.tick_once()
    assert_details(
        text="Tick 7 - task two finished",
        expected=py_trees.common.Status.SUCCESS,
        result=task_two.status,
    )
    assert(task_two.status == py_trees.common.Status.SUCCESS)

#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/splintered-reality/py_trees/devel/LICENSE
#
##############################################################################
# Imports
##############################################################################

import pytest

import py_trees
import py_trees.tests
import py_trees.console as console

##############################################################################
# Eternal Guard
##############################################################################


def create_tasks():
    return [
        py_trees.behaviours.Count(
            name="R-R-S",
            fail_until=0,
            running_until=2,
            success_until=3,
            reset=True
        ),
        py_trees.behaviours.Count(
            name="R-S",
            fail_until=0,
            running_until=1,
            success_until=100,
            reset=True
        ),
    ]


def impl_eternal_guard_checks(name, root, eternal_guard, tasks):
    console.banner(name)

    py_trees.tests.print_assert_banner()

    py_trees.tests.tick_tree(root, 1, 1, print_snapshot=True)
    print(console.green + "Tick 1: first guard fails, eternal guard fails" + console.reset)
    py_trees.tests.print_assert_details("eternal_guard", py_trees.common.Status.FAILURE, eternal_guard.status)
    assert(eternal_guard.status == py_trees.common.Status.FAILURE)

    py_trees.tests.tick_tree(root, 2, 2, print_snapshot=True)
    print(console.green + "Tick 2: guard checks ok, task sequence is running" + console.reset)
    py_trees.tests.print_assert_details("eternal_guard", py_trees.common.Status.RUNNING, eternal_guard.status)
    assert(eternal_guard.status == py_trees.common.Status.RUNNING)

    py_trees.tests.tick_tree(root, 3, 3, print_snapshot=True)
    py_trees.tests.tick_tree(root, 4, 4, print_snapshot=True)
    py_trees.tests.tick_tree(root, 5, 5, print_snapshot=True)
    print(console.green + "Tick 5: guards still ok, task sequence finished" + console.reset)
    py_trees.tests.print_assert_details("eternal_guard", py_trees.common.Status.SUCCESS, eternal_guard.status)
    assert(eternal_guard.status == py_trees.common.Status.SUCCESS)

    py_trees.tests.tick_tree(root, 6, 6, print_snapshot=True)
    py_trees.tests.tick_tree(root, 7, 7, print_snapshot=True)
    print(console.green + "Tick 7: tasks are running again, but the first guard fails" + console.reset)
    py_trees.tests.print_assert_details("eternal_guard", py_trees.common.Status.FAILURE, eternal_guard.status)
    assert(eternal_guard.status == py_trees.common.Status.FAILURE)
    for task in tasks:
        py_trees.tests.print_assert_details(task.name, py_trees.common.Status.INVALID, task.status)
        assert(task.status == py_trees.common.Status.INVALID)


def test_eternal_guard_sequence():
    """
    Test with a sequence that has no-memory.
    """
    root = py_trees.composites.Selector(name="Root", memory=False)
    eternal_guard = py_trees.composites.Sequence(name="Eternal Guard", memory=False)
    conditions = py_trees.composites.Parallel(
        name="Conditions",
        policy=py_trees.common.ParallelPolicy.SuccessOnOne()
    )
    frssssf = py_trees.behaviours.Count(
        name="F-R-S-S-S-S-F",
        fail_until=1,
        running_until=2,
        success_until=6,
        reset=False
    )
    success = py_trees.behaviours.Success(name="Success")
    task_sequence = py_trees.composites.Sequence(name="Task Sequence", memory=True)
    tasks = create_tasks()
    idle = py_trees.behaviours.Running()

    root.add_children([eternal_guard, idle])
    eternal_guard.add_children([conditions, task_sequence])
    task_sequence.add_children(tasks)
    conditions.add_children([frssssf, success])

    impl_eternal_guard_checks(
        name="Eternal Guard Idiom",
        root=root,
        eternal_guard=eternal_guard,
        tasks=tasks,
    )


def test_eternal_guard_decorator():
    root = py_trees.composites.Selector(name="Root", memory=False)

    def condition_success():
        return True

    # emulate py_trees.behaviours.Count
    class Count(object):
        def __init__(self):
            self.results = [
                py_trees.common.Status.FAILURE,
                py_trees.common.Status.SUCCESS,
                py_trees.common.Status.SUCCESS,
                py_trees.common.Status.SUCCESS,
                py_trees.common.Status.SUCCESS,
                py_trees.common.Status.SUCCESS,
                py_trees.common.Status.FAILURE,
            ]

        def condition(self, blackboard):
            try:
                new_result = self.results.pop(0)
                return new_result
            except IndexError:
                return py_trees.common.Status.FAILURE

    tasks = create_tasks()
    task_sequence = py_trees.composites.Sequence(name="Task Sequence", memory=True)
    task_sequence.add_children(tasks)
    idle = py_trees.behaviours.Running()
    nested_guard = py_trees.decorators.EternalGuard(
        name="Nested Guard",
        child=task_sequence,
        condition=condition_success
    )
    count = Count()
    eternal_guard = py_trees.decorators.EternalGuard(
        name="Eternal Guard",
        child=nested_guard,
        condition=count.condition
    )
    root.add_children([eternal_guard, idle])

    impl_eternal_guard_checks(
        name="Eternal Guard Decorator",
        root=root,
        eternal_guard=eternal_guard,
        tasks=tasks
    )

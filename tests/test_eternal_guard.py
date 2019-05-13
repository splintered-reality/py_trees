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
# Eternal Guard
##############################################################################


def create_tasks():
    return [
        py_trees.behaviours.Count(
            name="R-R-S",
            fail_until=0,
            running_until=2,
            success_until=100,
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

    py_trees.tests.tick_tree(root, 3, 5, print_snapshot=True)
    print(console.green + "Tick 5: guards still ok, task sequence finished" + console.reset)
    py_trees.tests.print_assert_details("eternal_guard", py_trees.common.Status.SUCCESS, eternal_guard.status)
    assert(eternal_guard.status == py_trees.common.Status.SUCCESS)

    py_trees.tests.tick_tree(root, 6, 7, print_snapshot=True)
    print(console.green + "Tick 7: tasks are running again, but the first guard fails" + console.reset)
    py_trees.tests.print_assert_details("eternal_guard", py_trees.common.Status.FAILURE, eternal_guard.status)
    assert(eternal_guard.status == py_trees.common.Status.FAILURE)
    for task in tasks:
        py_trees.tests.print_assert_details(task.name, py_trees.common.Status.INVALID, task.status)
        assert(task.status == py_trees.common.Status.INVALID)


def test_eternal_guard_idiom():
    root = py_trees.composites.Selector(name="Root")
    guards = [
        py_trees.behaviours.Count(
            name="F-S-S-S-S-S-F",  # TODO: F-R-S-S-S-S-F once I have that working
            fail_until=1,
            running_until=1,
            success_until=6,
            reset=False
        ),
        py_trees.behaviours.Success(name="Success")
    ]
    tasks = create_tasks()
    idle = py_trees.behaviours.Running()
    eternal_guard = py_trees.idioms.eternal_guard(name="Eternal Guard", guards=guards, tasks=tasks)

    root.add_children([eternal_guard, idle])

    impl_eternal_guard_checks(
        name="Eternal Guard Idiom",
        root=root,
        eternal_guard=eternal_guard,
        tasks=tasks
    )


def test_eternal_guard_decorator():
    pass
    root = py_trees.composites.Selector(name="Root")

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

        def condition(self):
            try:
                new_result = self.results.pop(0)
                return new_result
            except IndexError:
                return py_trees.common.Status.FAILURE

    tasks = create_tasks()
    task_sequence = py_trees.composites.Sequence("Task Sequence")
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

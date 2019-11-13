#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/splintered-reality/py_trees/devel/LICENSE
#
##############################################################################
# Imports
##############################################################################

import nose

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


def generate_eternal_guard():
    conditions = [
        py_trees.behaviours.Count(
            name="F-R-S-S-S-S-F",
            fail_until=1,
            running_until=2,
            success_until=6,
            reset=False
        ),
        py_trees.behaviours.Success(name="Success")
    ]
    tasks = create_tasks()
    task_sequence = py_trees.composites.Sequence("Task Sequence")
    task_sequence.add_children(tasks)
    eternal_guard = py_trees.idioms.eternal_guard(
        name="Eternal Guard",
        conditions=conditions,
        subtree=task_sequence
    )
    return eternal_guard, tasks


def test_eternal_guard_idiom():
    root = py_trees.composites.Selector(name="Root")
    eternal_guard, tasks = generate_eternal_guard()
    idle = py_trees.behaviours.Running()

    root.add_children([eternal_guard, idle])

    impl_eternal_guard_checks(
        name="Eternal Guard Idiom",
        root=root,
        eternal_guard=eternal_guard,
        tasks=tasks
    )


def test_eternal_guard_unique_names():
    py_trees.tests.clear_blackboard()
    blackboard = py_trees.blackboard.Client()
    for key in {"eternal_guard_condition_1", "eternal_guard_condition_2"}:
        blackboard.register_key(key=key, access=py_trees.common.Access.READ)
    for key in {"eternal_guard_condition_1"}:
        blackboard.register_key(key=key, access=py_trees.common.Access.WRITE)
    message = "Ha, stole it"
    blackboard.eternal_guard_condition_1 = message
    root = py_trees.composites.Selector(name="Root")
    eternal_guard, unused_tasks = generate_eternal_guard()
    root.add_children([eternal_guard])
    # tick once, get variables on the blackboard
    py_trees.tests.tick_tree(root, 1, 1, print_snapshot=True)
    assert(blackboard.get("eternal_guard_condition_1") == message)  # wasn't overwritten
    with nose.tools.assert_raises_regexp(KeyError, "exist"):
        print("Expecting a KeyError with substring 'yet exist'")
        unused = blackboard.eternal_guard_condition_2


def test_eternal_guard_decorator():
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

        def condition(self, blackboard):
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

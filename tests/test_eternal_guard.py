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
# Helpers
##############################################################################


def print_assert_banner():
    print(console.green + "----- Asserts -----" + console.reset)


def print_assert_details(text, expected, result):
    print(console.green + text +
          "." * (70 - len(text)) +
          console.cyan + "{}".format(expected) +
          console.yellow + " [{}]".format(result) +
          console.reset)

##############################################################################
# Eternal Guard
##############################################################################


def test_mirror():
    console.banner("Mirror")

    root = py_trees.composites.Selector(name="Root")
    guards = [
        py_trees.behaviours.Count(
            name="F-S-S-S-S-S-F",
            fail_until=1,
            running_until=1,
            success_until=6,
            reset=False
        ),
        py_trees.behaviours.Success(name="Success")
    ]
    tasks = [
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
    idle = py_trees.behaviours.Running()
    eternal_guard = py_trees.idioms.eternal_guard(name="Eternal Guard", guards=guards, tasks=tasks)

    root.add_children([eternal_guard, idle])

    print_assert_banner()

    py_trees.tests.tick_tree(root, 1, 1, print_snapshot=True)
    print(console.green + "Tick 1: first guard fails, eternal guard fails" + console.reset)
    print_assert_details("eternal_guard", py_trees.common.Status.FAILURE, eternal_guard.status)
    assert(eternal_guard.status == py_trees.common.Status.FAILURE)

    py_trees.tests.tick_tree(root, 2, 2, print_snapshot=True)
    print(console.green + "Tick 2: guard checks ok, task sequence is running" + console.reset)
    print_assert_details("eternal_guard", py_trees.common.Status.RUNNING, eternal_guard.status)
    assert(eternal_guard.status == py_trees.common.Status.RUNNING)

    py_trees.tests.tick_tree(root, 3, 5, print_snapshot=True)
    print(console.green + "Tick 5: guards still ok, task sequence finished" + console.reset)
    print_assert_details("eternal_guard", py_trees.common.Status.SUCCESS, eternal_guard.status)
    assert(eternal_guard.status == py_trees.common.Status.SUCCESS)

    py_trees.tests.tick_tree(root, 6, 7, print_snapshot=True)
    print(console.green + "Tick 7: tasks are running again, but the first guard fails" + console.reset)
    print_assert_details("eternal_guard", py_trees.common.Status.FAILURE, eternal_guard.status)
    assert(eternal_guard.status == py_trees.common.Status.FAILURE)
    for task in tasks:
        print_assert_details(task.name, py_trees.common.Status.INVALID, task.status)
        assert(task.status == py_trees.common.Status.INVALID)

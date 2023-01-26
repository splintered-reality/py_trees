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


def test_high_priority_interrupt() -> None:
    console.banner("High Priority Interrupt")
    task_one = py_trees.behaviours.StatusQueue(
        name="Task 1",
        queue=[
            py_trees.common.Status.RUNNING,
            py_trees.common.Status.RUNNING,
        ],
        eventually=py_trees.common.Status.SUCCESS,
    )
    task_two = py_trees.behaviours.StatusQueue(
        name="Task 2",
        queue=[
            py_trees.common.Status.RUNNING,
            py_trees.common.Status.RUNNING,
        ],
        eventually=py_trees.common.Status.SUCCESS,
    )
    tasks = [task_one, task_two]
    high_priority_interrupt = py_trees.decorators.RunningIsFailure(
        name="High Priority", child=py_trees.behaviours.Periodic(name="Periodic", n=3)
    )
    piwylo = py_trees.idioms.pick_up_where_you_left_off(
        name="Pick Up\nWhere You\nLeft Off", tasks=tasks
    )
    root = py_trees.composites.Selector(name="Root", memory=False)
    root.add_children([high_priority_interrupt, piwylo])

    print(py_trees.display.unicode_tree(root))
    visitor = py_trees.visitors.DebugVisitor()
    py_trees.tests.tick_tree(root, 1, 3, visitors=[visitor])
    print()

    print("\n--------- Assertions ---------\n")
    print("high_priority_interrupt.status == py_trees.common.Status.FAILURE")
    assert high_priority_interrupt.status == py_trees.common.Status.FAILURE
    print("piwylo.status == py_trees.common.Status.RUNNING")
    assert piwylo.status == py_trees.common.Status.RUNNING
    print("task_one.status == py_trees.common.Status.SUCCESS")
    assert task_one.status == py_trees.common.Status.SUCCESS
    print("task_two.status == py_trees.common.Status.RUNNING")
    assert task_two.status == py_trees.common.Status.RUNNING

    py_trees.tests.tick_tree(root, 4, 5, visitors=[visitor])

    print("\n--------- Assertions ---------\n")
    print("high_priority_interrupt.status == py_trees.common.Status.SUCCESS")
    assert high_priority_interrupt.status == py_trees.common.Status.SUCCESS
    print("piwylo.status == py_trees.common.Status.INVALID")
    assert piwylo.status == py_trees.common.Status.INVALID
    print("task_one.status == py_trees.common.Status.INVALID")
    assert task_one.status == py_trees.common.Status.INVALID
    print("task_two.status == py_trees.common.Status.INVALID")
    assert task_two.status == py_trees.common.Status.INVALID

    py_trees.tests.tick_tree(root, 6, 8, visitors=[visitor])

    print("\n--------- Assertions ---------\n")
    print("high_priority_interrupt.status == py_trees.common.Status.FAILURE")
    assert high_priority_interrupt.status == py_trees.common.Status.FAILURE
    print("piwylo.status == py_trees.common.Status.RUNNING")
    assert piwylo.status == py_trees.common.Status.RUNNING
    print("task_one.status == py_trees.common.Status.INVALID")
    assert task_one.status == py_trees.common.Status.INVALID
    print("task_two.status == py_trees.common.Status.RUNNING")
    assert task_two.status == py_trees.common.Status.RUNNING

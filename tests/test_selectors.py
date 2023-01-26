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
logger = py_trees.logging.Logger("Tests")

##############################################################################
# Tests
##############################################################################


def test_tick_running_with_no_memory() -> None:
    console.banner("Tick-Running with No Memory")
    py_trees.tests.print_assert_banner()
    root = py_trees.composites.Selector(name="Selector w/o Memory", memory=False)
    child_1 = py_trees.behaviours.StatusQueue(
        name="Fail Fast",
        queue=[py_trees.common.Status.FAILURE],
        eventually=py_trees.common.Status.SUCCESS,
    )
    child_2 = py_trees.behaviours.Running(name="Running")
    root.add_children([child_1, child_2])
    root.tick_once()
    print(py_trees.display.unicode_tree(root, show_status=True))
    py_trees.tests.print_assert_details(
        "1::Selector Status", py_trees.common.Status.RUNNING, root.status
    )
    assert root.status == py_trees.common.Status.RUNNING
    py_trees.tests.print_assert_details(
        "2::Child 1 Status", py_trees.common.Status.FAILURE, child_1.status
    )
    assert child_1.status == py_trees.common.Status.FAILURE
    py_trees.tests.print_assert_details(
        "2::Child 2 Status", py_trees.common.Status.RUNNING, child_2.status
    )
    assert child_2.status == py_trees.common.Status.RUNNING
    root.tick_once()
    print(py_trees.display.unicode_tree(root, show_status=True))
    py_trees.tests.print_assert_details(
        "2::Selector Status", py_trees.common.Status.SUCCESS, root.status
    )
    assert root.status == py_trees.common.Status.SUCCESS
    py_trees.tests.print_assert_details(
        "2::Child 1 Status", py_trees.common.Status.SUCCESS, child_1.status
    )
    assert child_1.status == py_trees.common.Status.SUCCESS
    py_trees.tests.print_assert_details(
        "2::Child 2 Status", py_trees.common.Status.INVALID, child_2.status
    )
    assert child_2.status == py_trees.common.Status.INVALID


def test_with_memory_priority_handling() -> None:
    # This test should check two things:
    # 1) Skipped higher priorities are set to INVALID
    # 2) On a running behaviour's eventual FAILURE, it proceeds to the next lower priority
    console.banner("Tick-Running with Memory - Priority Handling")
    py_trees.tests.print_assert_banner()
    root = py_trees.composites.Selector(name="Selector w/ Memory", memory=True)
    child_1 = py_trees.behaviours.Failure("Failure")
    child_2 = py_trees.behaviours.StatusQueue(
        name="Run-Fail",
        queue=[py_trees.common.Status.RUNNING, py_trees.common.Status.FAILURE],
        eventually=None,
    )
    child_3 = py_trees.behaviours.Success(name="Success")
    root.add_children([child_1, child_2, child_3])
    root.tick_once()
    print(py_trees.display.unicode_tree(root, show_status=True))
    py_trees.tests.print_assert_details(
        "1::Selector Status", py_trees.common.Status.RUNNING, root.status
    )
    assert root.status == py_trees.common.Status.RUNNING
    py_trees.tests.print_assert_details(
        "1::Child 1 Status", py_trees.common.Status.FAILURE, child_1.status
    )
    assert child_1.status == py_trees.common.Status.FAILURE
    py_trees.tests.print_assert_details(
        "1::Child 2 Status", py_trees.common.Status.RUNNING, child_2.status
    )
    assert child_2.status == py_trees.common.Status.RUNNING
    py_trees.tests.print_assert_details(
        "1::Child 3 Status", py_trees.common.Status.INVALID, child_3.status
    )
    assert child_3.status == py_trees.common.Status.INVALID
    root.tick_once()
    print(py_trees.display.unicode_tree(root, show_status=True))
    py_trees.tests.print_assert_details(
        "1::Selector Status", py_trees.common.Status.SUCCESS, root.status
    )
    assert root.status == py_trees.common.Status.SUCCESS
    py_trees.tests.print_assert_details(
        "2::Child 1 Status", py_trees.common.Status.INVALID, child_1.status
    )
    assert child_1.status == py_trees.common.Status.INVALID
    py_trees.tests.print_assert_details(
        "2::Child 2 Status", py_trees.common.Status.FAILURE, child_2.status
    )
    assert child_2.status == py_trees.common.Status.FAILURE
    py_trees.tests.print_assert_details(
        "2::Child 3 Status", py_trees.common.Status.SUCCESS, child_3.status
    )
    assert child_3.status == py_trees.common.Status.SUCCESS


##############################################################################
# Tests - Dynamic Insertion / Removal
##############################################################################


def test_tick_add_with_current_child() -> None:
    console.banner("Tick-Add with Current Child")
    py_trees.tests.print_assert_banner()
    root = py_trees.composites.Selector(name="Selector", memory=False)
    root.tick_once()
    child = py_trees.behaviours.Failure(name="Failure")
    print(py_trees.display.unicode_tree(root, show_status=True))
    root.add_child(child)
    print(py_trees.display.unicode_tree(root, show_status=True))
    py_trees.tests.print_assert_details("Current Child", None, root.current_child)
    assert root.current_child is None


def test_add_tick_add_with_current_child() -> None:
    console.banner("Add-Tick-Add with Current Child")
    py_trees.tests.print_assert_banner()
    root = py_trees.composites.Selector(name="Selector", memory=False)
    run1 = py_trees.behaviours.Running("Run1")
    run2 = py_trees.behaviours.Running("Run2")
    root.add_child(run1)
    root.tick_once()
    assert root.current_child is not None
    print(py_trees.display.unicode_tree(root, show_status=True))
    py_trees.tests.print_assert_details(
        "Current Child", run1.name, root.current_child.name
    )
    assert root.current_child.name == run1.name
    root.add_child(run2)
    assert root.current_child is not None
    print(py_trees.display.unicode_tree(root, show_status=True))
    py_trees.tests.print_assert_details(
        "Current Child", run1.name, root.current_child.name
    )
    assert root.current_child.name == run1.name


def test_add_tick_insert_with_current_child() -> None:
    console.banner("Add-Tick-Insert with Current Child")
    py_trees.tests.print_assert_banner()
    root = py_trees.composites.Selector(name="Selector", memory=False)
    run1 = py_trees.behaviours.Running("Run1")
    run2 = py_trees.behaviours.Running("Run2")
    root.add_child(run1)
    root.tick_once()
    assert root.current_child is not None
    print(py_trees.display.unicode_tree(root, show_status=True))
    py_trees.tests.print_assert_details(
        "Current Child", run1.name, root.current_child.name
    )
    assert root.current_child.name == run1.name
    root.insert_child(run2, index=0)
    assert root.current_child is not None
    print(py_trees.display.unicode_tree(root, show_status=True))
    py_trees.tests.print_assert_details(
        "Current Child", run1.name, root.current_child.name
    )
    assert root.current_child.name == run1.name


def test_add_tick_remove_with_current_child() -> None:
    console.banner("Add-Tick-Remove with Current Child")
    py_trees.tests.print_assert_banner()
    root = py_trees.composites.Selector(name="Selector", memory=False)
    child = py_trees.behaviours.Success(name="Success")
    root.add_child(child)
    root.tick_once()
    print(py_trees.display.unicode_tree(root, show_status=True))
    root.remove_child(child)
    print(py_trees.display.unicode_tree(root, show_status=True))
    py_trees.tests.print_assert_details("Current Child", None, root.current_child)
    assert root.current_child is None

    root.add_child(child)
    root.tick_once()
    print(py_trees.display.unicode_tree(root, show_status=True))
    root.remove_all_children()
    print(py_trees.display.unicode_tree(root, show_status=True))
    py_trees.tests.print_assert_details("Current Child", None, root.current_child)
    assert root.current_child is None

    replacement = py_trees.behaviours.Success(name="Replacement")
    root.add_child(child)
    root.tick_once()
    print(py_trees.display.unicode_tree(root, show_status=True))
    root.replace_child(child=child, replacement=replacement)
    print(py_trees.display.unicode_tree(root, show_status=True))
    py_trees.tests.print_assert_details("Current Child", None, root.current_child)
    assert root.current_child is None

    root.remove_all_children()
    root.add_child(child)
    root.tick_once()
    print(py_trees.display.unicode_tree(root, show_status=True))
    root.remove_child_by_id(child_id=child.id)
    print(py_trees.display.unicode_tree(root, show_status=True))
    py_trees.tests.print_assert_details("Current Child", None, root.current_child)
    assert root.current_child is None

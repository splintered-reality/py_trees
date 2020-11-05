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
import typing

##############################################################################
# Logging Level
##############################################################################

py_trees.logging.level = py_trees.logging.Level.DEBUG
logger = py_trees.logging.Logger("Nosetest")

##############################################################################
# Helpers
##############################################################################


def assert_banner():
    print(console.green + "----- Asserts -----" + console.reset)


def assert_details(text, expected, result):
    print(console.green + text +
          "." * (70 - len(text)) +
          console.cyan + "{}".format(expected) +
          console.yellow + " [{}]".format(result) +
          console.reset)

##############################################################################
# Tests
##############################################################################


def test_static_sequence_successes():
    console.banner('Static Sequence Successes')
    assert_banner()

    success1 = py_trees.behaviours.Success(name="Success1")
    success2 = py_trees.behaviours.Success(name="Success2")
    success3 = py_trees.behaviours.Success(name="Success3")
    success4 = py_trees.behaviours.Success(name="Success4")

    children = [
        success1,
        success2,
        success3,
        success4,
    ]
    root = py_trees.composites.Sequence(name="Sequence", children=children)

    root.tick_once()

    assert_details("Current Child", success4.name, root.current_child.name)
    assert root.tip().name == success4.name, 'should execute all 4 nodes of this Sequence'


def test_static_sequence_with_failure():
    console.banner('Static Sequence With Failure')
    assert_banner()

    success1 = py_trees.behaviours.Success(name="Success1")
    success2 = py_trees.behaviours.Success(name="Success2")
    failure = py_trees.behaviours.Failure(name="Failure")
    success3 = py_trees.behaviours.Success(name="Success3")

    children = [
        success1,
        success2,
        failure,
        success3,
    ]
    root = py_trees.composites.Sequence(name="Sequence", children=children)

    root.tick_once()

    assert_details("Current Child", failure.name, root.current_child.name)
    assert root.tip().name == failure.name, 'should execute first 2 nodes of this Sequence and fail'


def test_tick_add_with_current_child():
    console.banner('Tick-Add with Current Child')
    assert_banner()
    root = py_trees.composites.Sequence(name="Sequence")
    root.tick_once()
    child = py_trees.behaviours.Failure(name="Failure")
    print(py_trees.display.unicode_tree(root, show_status=True))
    root.add_child(child)
    print(py_trees.display.unicode_tree(root, show_status=True))
    assert_details("Current Child", None, root.current_child)
    assert(root.current_child is None)


def test_add_tick_add_with_current_child():
    console.banner('Add-Tick-Add with Current Child')
    assert_banner()
    root = py_trees.composites.Sequence(name="Sequence")
    run1 = py_trees.behaviours.Running("Run1")
    run2 = py_trees.behaviours.Running("Run2")
    root.add_child(run1)
    root.tick_once()
    print(py_trees.display.unicode_tree(root, show_status=True))
    assert_details("Current Child", run1.name, root.current_child.name)
    assert(root.current_child.name == run1.name)
    root.add_child(run2)
    print(py_trees.display.unicode_tree(root, show_status=True))
    assert_details("Current Child", run1.name, root.current_child.name)
    assert(root.current_child.name == run1.name)


def test_add_tick_insert_with_current_child():
    console.banner('Add-Tick-Insert with Current Child')
    assert_banner()
    root = py_trees.composites.Sequence(name="Sequence")
    run1 = py_trees.behaviours.Running("Run1")
    run2 = py_trees.behaviours.Running("Run2")
    root.add_child(run1)
    root.tick_once()
    print(py_trees.display.unicode_tree(root, show_status=True))
    assert_details("Current Child", run1.name, root.current_child.name)
    assert(root.current_child.name == run1.name)
    root.insert_child(run2, index=0)
    print(py_trees.display.unicode_tree(root, show_status=True))
    assert_details("Current Child", run1.name, root.current_child.name)
    assert(root.current_child.name == run1.name)


def test_add_tick_remove_with_current_child():
    console.banner('Add-Tick-Remove with Current Child')
    assert_banner()
    root = py_trees.composites.Sequence(name="Sequence")
    child = py_trees.behaviours.Success(name="Success")
    root.add_child(child)
    root.tick_once()
    print(py_trees.display.unicode_tree(root, show_status=True))
    root.remove_child(child)
    print(py_trees.display.unicode_tree(root, show_status=True))
    assert_details("Current Child", None, root.current_child)
    assert(root.current_child is None)

    root.add_child(child)
    root.tick_once()
    print(py_trees.display.unicode_tree(root, show_status=True))
    root.remove_all_children()
    print(py_trees.display.unicode_tree(root, show_status=True))
    assert_details("Current Child", None, root.current_child)
    assert(root.current_child is None)

    replacement = py_trees.behaviours.Success(name="Replacement")
    root.add_child(child)
    root.tick_once()
    print(py_trees.display.unicode_tree(root, show_status=True))
    root.replace_child(child=child, replacement=replacement)
    print(py_trees.display.unicode_tree(root, show_status=True))
    assert_details("Current Child", None, root.current_child)
    assert(root.current_child is None)

    root.remove_all_children()
    root.add_child(child)
    root.tick_once()
    print(py_trees.display.unicode_tree(root, show_status=True))
    root.remove_child_by_id(child_id=child.id)
    print(py_trees.display.unicode_tree(root, show_status=True))
    assert_details("Current Child", None, root.current_child)
    assert(root.current_child is None)

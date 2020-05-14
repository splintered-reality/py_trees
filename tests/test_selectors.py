#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/splintered-reality/py_trees/devel/LICENSE
#

##############################################################################
# Imports
##############################################################################
import itertools
import nose.tools
import py_trees
import py_trees.console as console

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


def test_tick_add_with_current_child():
    console.banner('Tick-Add with Current Child')
    assert_banner()
    root = py_trees.composites.Selector(name="Selector")
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
    root = py_trees.composites.Selector(name="Selector")
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
    root = py_trees.composites.Selector(name="Selector")
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
    root = py_trees.composites.Selector(name="Selector")
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

#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/splintered-reality/py_trees/devel/LICENSE
#

##############################################################################
# Imports
##############################################################################

import nose.tools

import py_trees
import py_trees.console as console

from nose.tools import assert_raises

##############################################################################
# Logging Level
##############################################################################

py_trees.logging.level = py_trees.logging.Level.DEBUG
logger = py_trees.logging.Logger("Nosetest")

##############################################################################
# Tests
##############################################################################


def test_replacing_children():
    console.banner("Replacing Children")
    parent = py_trees.composites.Sequence(name="Parent")
    old_child = py_trees.behaviours.Success(name="Old Child")
    new_child = py_trees.behaviours.Success(name="New Child")
    parent.add_child(old_child)
    parent.replace_child(old_child, new_child)
    print("\n--------- Assertions ---------\n")
    print("old_child.parent is None")
    assert(old_child.parent is None)
    print("new_child.parent is parent")
    assert(new_child.parent is parent)


def test_removing_children():
    console.banner("Removing Children")
    parent = py_trees.composites.Sequence(name="Parent")
    child = py_trees.behaviours.Success(name="Child")
    print("\n--------- Assertions ---------\n")
    print("child.parent is None after removing by instance")
    parent.add_child(child)
    parent.remove_child(child)
    assert(child.parent is None)
    print("child.parent is None after removing by id")
    parent.add_child(child)
    parent.remove_child_by_id(child.id)
    assert(child.parent is None)
    print("child.parent is None after removing all children")
    parent.add_child(child)
    parent.remove_all_children()
    assert(child.parent is None)


def test_composite_errors():
    console.banner("Timer Errors")
    root = py_trees.composites.Selector()
    print("add_child raises a 'TypeError' due to not being an instance of Behaviour")
    with nose.tools.assert_raises(TypeError) as context:
        root.add_child(5.0)
        print("TypeError has message with substring 'behaviours'")
        assert("behaviours" in str(context.exception))


def test_protect_against_multiple_parents():
    console.banner("Protect Against Multiple Parents")
    child = py_trees.behaviours.Success()
    first_parent = py_trees.composites.Selector()
    second_parent = py_trees.composites.Sequence()
    with nose.tools.assert_raises(RuntimeError):
        print("Adding a child to two parents")
        print("Expecting a RuntimeError")
        for parent in [first_parent, second_parent]:
            parent.add_child(child)

#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/splintered-reality/py_trees/devel/LICENSE
#

##############################################################################
# Imports
##############################################################################

import typing

import py_trees
import py_trees.console as console
import pytest

##############################################################################
# Logging Level
##############################################################################

py_trees.logging.level = py_trees.logging.Level.DEBUG
logger = py_trees.logging.Logger("Tests")

##############################################################################
# Helpers
##############################################################################


def assert_banner() -> None:
    print(console.green + "----- Asserts -----" + console.reset)


AssertResultType = typing.TypeVar("AssertResultType")


def assert_details(
    text: str, expected: AssertResultType, result: AssertResultType
) -> None:
    print(
        console.green
        + text
        + "." * (70 - len(text))
        + console.cyan
        + "{}".format(expected)
        + console.yellow
        + " [{}]".format(result)
        + console.reset
    )


##############################################################################
# Tests
##############################################################################


def test_replacing_children() -> None:
    console.banner("Replacing Children")
    parent = py_trees.composites.Sequence(name="Parent", memory=True)
    front = py_trees.behaviours.Success(name="Front")
    back = py_trees.behaviours.Success(name="Back")
    old_child = py_trees.behaviours.Success(name="Old Child")
    new_child = py_trees.behaviours.Success(name="New Child")
    parent.add_children([front, old_child, back])
    print(py_trees.display.unicode_tree(parent, show_status=True))
    parent.replace_child(old_child, new_child)
    print(py_trees.display.unicode_tree(parent, show_status=True))
    print("\n--------- Assertions ---------\n")
    print("old_child.parent is None")
    assert old_child.parent is None
    print("new_child.parent is parent")
    assert new_child.parent is parent


def test_removing_children() -> None:
    console.banner("Removing Children")
    parent = py_trees.composites.Sequence(name="Parent", memory=True)
    child = py_trees.behaviours.Success(name="Child")
    print("\n--------- Assertions ---------\n")
    print("child.parent is None after removing by instance")
    parent.add_child(child)
    parent.remove_child(child)
    assert child.parent is None
    print("child.parent is None after removing by id")
    parent.add_child(child)
    parent.remove_child_by_id(child.id)
    assert child.parent is None
    print("child.parent is None after removing all children")
    parent.add_child(child)
    parent.remove_all_children()
    assert child.parent is None


def test_composite_add_child_exception() -> None:
    console.banner("Composite Add Child Exception - Invalid Type")
    root = py_trees.composites.Selector(name="Selector", memory=False)
    with pytest.raises(TypeError) as context:  # if raised, context survives
        # intentional error - silence mypy
        root.add_child(5.0)  # type: ignore[arg-type]
        py_trees.tests.print_assert_details("TypeError raised", "raised", "not raised")
    py_trees.tests.print_assert_details("TypeError raised", "yes", "yes")
    assert "TypeError" == context.typename
    py_trees.tests.print_assert_details(
        "Substring match", "behaviours", f"{context.value}"
    )
    assert "behaviours" in str(context.value)


def test_protect_against_multiple_parents() -> None:
    console.banner("Protect Against Multiple Parents")
    child = py_trees.behaviours.Success(name="Success")
    first_parent = py_trees.composites.Selector(name="Selector", memory=False)
    second_parent = py_trees.composites.Sequence(name="Sequence", memory=True)
    with pytest.raises(RuntimeError) as context:  # if raised, context survives
        # Adding a child to two parents - expecting a RuntimeError
        for parent in [first_parent, second_parent]:
            parent.add_child(child)
        py_trees.tests.print_assert_details(
            "RuntimeError raised", "raised", "not raised"
        )
    py_trees.tests.print_assert_details("RuntimeError raised", "yes", "yes")
    assert "RuntimeError" == context.typename
    py_trees.tests.print_assert_details("Substring match", "parent", f"{context.value}")
    assert "parent" in str(context.value)


def test_remove_nonexistant_child() -> None:
    console.banner("Remove non-existant child")
    root = py_trees.composites.Sequence(name="Sequence", memory=True)
    child = py_trees.behaviours.Success(name="Success")
    root.add_child(child)
    non_existant_child = py_trees.behaviours.Success(name="Ooops")

    with pytest.raises(IndexError) as context:  # if raised, context survives
        root.remove_child_by_id(non_existant_child.id)
        py_trees.tests.print_assert_details("IndexError raised", "raised", "not raised")
    py_trees.tests.print_assert_details("IndexError raised", "yes", "yes")
    assert "IndexError" == context.typename
    py_trees.tests.print_assert_details(
        "Substring match", "not found", f"{context.value}"
    )
    assert "not found" in str(context.value)

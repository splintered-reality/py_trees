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
import operator

from py_trees.common import Status
from py_trees.behaviours import (
    CheckBlackboardVariableExists,
    WaitForBlackboardVariable
)
from py_trees.blackboard import Blackboard, Client

##############################################################################
# Logging Level
##############################################################################

# py_trees.logging.level = py_trees.logging.Level.DEBUG
# logger = py_trees.logging.Logger("Nosetest")

##############################################################################
# Helpers
##############################################################################


def assert_banner():
    print(console.green + "----- Asserts -----" + console.reset)


def assert_details(text, expected, result):
    print(console.green + text +
          "." * (40 - len(text)) +
          console.cyan + "{}".format(expected) +
          console.yellow + " [{}]".format(result) +
          console.reset)


class Nested(object):

    def __init__(self):
        self.foo = 'bar'


def create_blackboard():
    """
    Create a blackboard with a few variables.

    Fill with as many different types as we need to get full coverage on
    pretty printing blackboard tests.
    """
    Blackboard.clear()
    blackboard = Client(name="Tester")
    for key in {"foo", "some_tuple", "nested", "nothing"}:
        blackboard.register_key(
            key=key,
            access=py_trees.common.Access.READ
        )
    for key in {"foo", "some_tuple", "nested", "nothing"}:
        blackboard.register_key(
            key=key,
            access=py_trees.common.Access.WRITE
        )
    blackboard.foo = "bar"
    blackboard.some_tuple = (1, "bar")
    blackboard.nested = Nested()
    blackboard.nothing = None
    return blackboard

##############################################################################
# Tests
##############################################################################


def test_variable_exists():
    console.banner("Check Existence of Variable")
    unused_client = create_blackboard()
    tuples = []
    tuples.append((CheckBlackboardVariableExists(
        name="check_foo_exists", variable_name="foo"), Status.SUCCESS))
    tuples.append((CheckBlackboardVariableExists(
        name="check_bar_exists", variable_name="bar"), Status.FAILURE))
    tuples.append((CheckBlackboardVariableExists(
        name="check_nested_foo_exists", variable_name="nested.foo"), Status.SUCCESS))
    tuples.append((CheckBlackboardVariableExists(
        name="check_nested_bar_exists", variable_name="nested.bar"), Status.FAILURE))
    for b, unused in tuples:
        b.tick_once()
    assert_banner()
    for b, asserted_result in tuples:
        assert_details(
            text="looking for '{}'".format(b.variable_name),
            expected=asserted_result,
            result=b.status
        )
        assert(b.status == asserted_result)


def test_wait_for_variable():
    console.banner("Wait for Variable")
    unused_client = create_blackboard()
    tuples = []
    tuples.append((WaitForBlackboardVariable(
        name="Wait for Foo", variable_name="foo"), Status.SUCCESS))
    tuples.append((WaitForBlackboardVariable(
        name="Wait for Bar", variable_name="bar"), Status.RUNNING))
    tuples.append((WaitForBlackboardVariable(
        name="Wait for nested.foo", variable_name="nested.foo"), Status.SUCCESS))
    tuples.append((WaitForBlackboardVariable(
        name="Wait for nested.bar", variable_name="nested.bar"), Status.RUNNING))
    for b, unused in tuples:
        b.tick_once()
    assert_banner()
    for b, asserted_result in tuples:
        assert_details(
            text="waiting for '{}'".format(b.variable_name),
            expected=asserted_result,
            result=b.status
        )
        assert(b.status == asserted_result)


def test_unset_blackboard_variable():
    console.banner("Unset Blackboard Variable")
    blackboard = create_blackboard()
    blackboard.foo = "bar"
    clear_foo = py_trees.behaviours.UnsetBlackboardVariable(name="Clear Foo", key="foo")
    clear_bar = py_trees.behaviours.UnsetBlackboardVariable(name="Clear Bar", key="bar")
    py_trees.display.unicode_blackboard()
    assert_banner()
    assert_details(
        text="'foo' exists",
        expected=True,
        result="foo" in Blackboard.storage.keys()
    )
    assert("foo" in Blackboard.storage.keys())
    print("Ticking 'Clear Foo' once...")
    clear_foo.tick_once()
    assert_details(
        text="'foo' does not exist",
        expected=True,
        result="foo" not in Blackboard.storage.keys()
    )
    assert("foo" not in Blackboard.storage.keys())
    print("Ticking 'Clear Bar' once...")
    clear_bar.tick_once()
    assert_details(
        text="'bar' does not exist",
        expected=True,
        result="bar" not in Blackboard.storage.keys()
    )
    assert_details(
        text="'foo' still does not exist",
        expected=True,
        result="foo" not in Blackboard.storage.keys()
    )
    assert("foo" not in Blackboard.storage.keys())


def test_set_blackboard_variable():
    console.banner("Set Blackboard Variable")
    blackboard = create_blackboard()
    set_foo = py_trees.behaviours.SetBlackboardVariable(
        name="Set Foo",
        variable_name="foo",
        variable_value="bar"
    )
    conservative_set_foo = py_trees.behaviours.SetBlackboardVariable(
        name="Conservative Set Foo",
        variable_name="foo",
        variable_value="bar",
        overwrite=False
    )
    blackboard.unset("foo")
    assert_banner()
    set_foo.tick_once()
    assert_details(
        text="Set 'foo' (doesn't exist)",
        expected="bar",
        result=blackboard.foo
    )
    assert("bar" == blackboard.foo)
    blackboard.foo = "whoop"
    set_foo.tick_once()
    assert_details(
        text="Set 'foo' (exists)",
        expected="bar",
        result=blackboard.foo
    )
    blackboard.unset("foo")
    assert_details(
        text="Set 'foo' Status",
        expected=py_trees.common.Status.SUCCESS,
        result=set_foo.status
    )
    assert(set_foo.status == Status.SUCCESS)
    conservative_set_foo.tick_once()
    assert_details(
        text="Conservative 'foo' (doesn't exist)",
        expected="bar",
        result=blackboard.foo
    )
    assert("bar" == blackboard.foo)
    blackboard.foo = "whoop"
    conservative_set_foo.tick_once()
    assert_details(
        text="Conservative Set 'foo' (exists)",
        expected="whoop",
        result=blackboard.foo
    )
    assert("whoop" == blackboard.foo)
    assert_details(
        text="Conservative Set 'foo' Status",
        expected=py_trees.common.Status.FAILURE,
        result=conservative_set_foo.status
    )
    assert(conservative_set_foo.status == Status.FAILURE)

    nested_set_foo = py_trees.behaviours.SetBlackboardVariable(
        name="Nested Set Foo",
        variable_name="nested.foo",
        variable_value="dude",
        overwrite=True
    )
    nested_set_foo.tick_once()
    assert_details(
        text="Nested set foo (value)",
        expected="dude",
        result=blackboard.nested.foo
    )
    assert_details(
        text="Nested set foo (status)",
        expected=py_trees.common.Status.SUCCESS,
        result=nested_set_foo.status
    )
    blackboard.nested = 5
    nested_set_foo.tick_once()
    assert_details(
        text="Nested set foo, no nested attribute (status)",
        expected=py_trees.common.Status.FAILURE,
        result=nested_set_foo.status
    )


def test_check_variable_value():
    console.banner("Check Variable Value")
    unused_client = create_blackboard()
    tuples = []
    print(py_trees.display.unicode_blackboard())
    tuples.append((py_trees.behaviours.CheckBlackboardVariableValue(
        name="check_foo_equals_bar", variable_name="foo", expected_value="bar"), Status.SUCCESS))
    tuples.append((py_trees.behaviours.CheckBlackboardVariableValue(
        name="check_foo_equals_foo", variable_name="foo", expected_value="foo"), Status.FAILURE))
    tuples.append((py_trees.behaviours.CheckBlackboardVariableValue(
        name="check_non_existant_bar_equals_bar", variable_name="bar", expected_value="bar"), Status.FAILURE))
    tuples.append((py_trees.behaviours.CheckBlackboardVariableValue(
        name="check_nested_foo_equals_bar", variable_name="nested.foo", expected_value="bar"),
        Status.SUCCESS))
    tuples.append((py_trees.behaviours.CheckBlackboardVariableValue(
        name="check_nested_foo_equals_foo", variable_name="nested.foo", expected_value="foo"),
        Status.FAILURE))
    for b, unused in tuples:
        b.tick_once()
        print("Feedback message {}".format(b.feedback_message))
    print("")
    assert_banner()
    for b, asserted_result in tuples:
        assert_details(
            text=b.name,
            expected=asserted_result,
            result=b.status
        )
        assert(b.status == asserted_result)


def test_check_variable_value_inverted():
    console.banner("Check Variable Value Neq")
    unused_client = create_blackboard()
    tuples = []
    print(py_trees.display.unicode_blackboard())
    tuples.append((py_trees.behaviours.CheckBlackboardVariableValue(
        name="check_foo_not_equals_bar", variable_name="foo", expected_value="bar",
        comparison_operator=operator.ne), Status.FAILURE))
    tuples.append((py_trees.behaviours.CheckBlackboardVariableValue(
        name="check_foo_not_equals_foo", variable_name="foo", expected_value="foo",
        comparison_operator=operator.ne), Status.SUCCESS))
    tuples.append((py_trees.behaviours.CheckBlackboardVariableValue(
        name="check_non_existant_bar_not_equals_bar", variable_name="bar", expected_value="bar",
        comparison_operator=operator.ne), Status.FAILURE))
    tuples.append((py_trees.behaviours.CheckBlackboardVariableValue(
        name="check_nested_foo_not_equals_bar", variable_name="nested.foo", expected_value="bar",
        comparison_operator=operator.ne),
        Status.FAILURE))
    tuples.append((py_trees.behaviours.CheckBlackboardVariableValue(
        name="check_nested_foo_not_equals_foo", variable_name="nested.foo", expected_value="foo",
        comparison_operator=operator.ne),
        Status.SUCCESS))
    for b, unused in tuples:
        b.tick_once()
        print("Feedback message {}".format(b.feedback_message))
    print("")
    assert_banner()
    for b, asserted_result in tuples:
        assert_details(
            text=b.name,
            expected=asserted_result,
            result=b.status
        )
        assert(b.status == asserted_result)


def test_wait_for_variable_value():
    console.banner("Wait for Variable Value")
    unused_client = create_blackboard()
    tuples = []
    print(py_trees.display.unicode_blackboard())
    tuples.append((py_trees.behaviours.WaitForBlackboardVariableValue(
        name="check_foo_equals_bar", variable_name="foo", expected_value="bar"), Status.SUCCESS))
    tuples.append((py_trees.behaviours.WaitForBlackboardVariableValue(
        name="check_foo_equals_foo", variable_name="foo", expected_value="foo"), Status.RUNNING))
    tuples.append((py_trees.behaviours.WaitForBlackboardVariableValue(
        name="check_non_existant_bar_equals_bar", variable_name="bar", expected_value="bar"), Status.RUNNING))
    tuples.append((py_trees.behaviours.WaitForBlackboardVariableValue(
        name="check_nested_foo_equals_bar", variable_name="nested.foo", expected_value="bar"),
        Status.SUCCESS))
    tuples.append((py_trees.behaviours.WaitForBlackboardVariableValue(
        name="check_nested_foo_equals_foo", variable_name="nested.foo", expected_value="foo"),
        Status.RUNNING))
    for b, unused in tuples:
        b.tick_once()
        print("Feedback message {}".format(b.feedback_message))
    print("")
    assert_banner()
    for b, asserted_result in tuples:
        assert_details(
            text=b.name,
            expected=asserted_result,
            result=b.status
        )
        assert(b.status == asserted_result)


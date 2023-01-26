#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/splintered-reality/py_trees/devel/LICENSE
#

##############################################################################
# Imports
##############################################################################

import operator
import typing

import py_trees
import py_trees.console as console
import py_trees.tests
import pytest
from py_trees.behaviours import CheckBlackboardVariableExists, WaitForBlackboardVariable
from py_trees.blackboard import Blackboard, Client

from py_trees.common import Status

##############################################################################
# Logging Level
##############################################################################

# py_trees.logging.level = py_trees.logging.Level.DEBUG
# logger = py_trees.logging.Logger("Nosetest")

##############################################################################
# Helpers
##############################################################################


class Nested(object):
    def __init__(self) -> None:
        self.foo = "bar"


def create_blackboard() -> py_trees.blackboard.Client:
    """
    Create a blackboard with a few variables.

    Fill with as many different types as we need to get full coverage on
    pretty printing blackboard tests.
    """
    Blackboard.clear()
    blackboard = Client(name="Tester")
    for key in {"foo", "some_tuple", "nested", "nothing"}:
        blackboard.register_key(key=key, access=py_trees.common.Access.READ)
    for key in {"foo", "some_tuple", "nested", "nothing"}:
        blackboard.register_key(key=key, access=py_trees.common.Access.WRITE)
    blackboard.foo = "bar"
    blackboard.some_tuple = (1, "bar")
    blackboard.nested = Nested()
    blackboard.nothing = None
    return blackboard


##############################################################################
# Tests
##############################################################################


def test_variable_exists() -> None:
    console.banner("Check Existence of Variable")
    unused_client = create_blackboard()  # noqa: F841 [unused]
    tuples = []
    tuples.append(
        (
            CheckBlackboardVariableExists(name="check_foo_exists", variable_name="foo"),
            Status.SUCCESS,
        )
    )
    tuples.append(
        (
            CheckBlackboardVariableExists(name="check_bar_exists", variable_name="bar"),
            Status.FAILURE,
        )
    )
    tuples.append(
        (
            CheckBlackboardVariableExists(
                name="check_nested_foo_exists", variable_name="nested.foo"
            ),
            Status.SUCCESS,
        )
    )
    tuples.append(
        (
            CheckBlackboardVariableExists(
                name="check_nested_bar_exists", variable_name="nested.bar"
            ),
            Status.FAILURE,
        )
    )
    for b, unused in tuples:
        b.tick_once()
    py_trees.tests.print_assert_banner()
    for b, asserted_result in tuples:
        py_trees.tests.print_assert_details(
            text="looking for '{}'".format(b.variable_name),
            expected=asserted_result,
            result=b.status,
        )
        assert b.status == asserted_result


def test_wait_for_variable() -> None:
    console.banner("Wait for Variable")
    unused_client = create_blackboard()  # noqa: F841 [unused]
    tuples = []
    tuples.append(
        (
            WaitForBlackboardVariable(name="Wait for Foo", variable_name="foo"),
            Status.SUCCESS,
        )
    )
    tuples.append(
        (
            WaitForBlackboardVariable(name="Wait for Bar", variable_name="bar"),
            Status.RUNNING,
        )
    )
    tuples.append(
        (
            WaitForBlackboardVariable(
                name="Wait for nested.foo", variable_name="nested.foo"
            ),
            Status.SUCCESS,
        )
    )
    tuples.append(
        (
            WaitForBlackboardVariable(
                name="Wait for nested.bar", variable_name="nested.bar"
            ),
            Status.RUNNING,
        )
    )
    for b, unused in tuples:
        b.tick_once()
    py_trees.tests.print_assert_banner()
    for b, asserted_result in tuples:
        py_trees.tests.print_assert_details(
            text="waiting for '{}'".format(b.variable_name),
            expected=asserted_result,
            result=b.status,
        )
        assert b.status == asserted_result


def test_unset_blackboard_variable() -> None:
    console.banner("Unset Blackboard Variable")
    blackboard = create_blackboard()
    blackboard.foo = "bar"
    clear_foo = py_trees.behaviours.UnsetBlackboardVariable(name="Clear Foo", key="foo")
    clear_bar = py_trees.behaviours.UnsetBlackboardVariable(name="Clear Bar", key="bar")
    py_trees.display.unicode_blackboard()
    py_trees.tests.print_assert_banner()
    py_trees.tests.print_assert_details(
        text="'/foo' exists", expected=True, result="/foo" in Blackboard.storage.keys()
    )
    assert "/foo" in Blackboard.storage.keys()
    print("Ticking 'Clear Foo' once...")
    clear_foo.tick_once()
    py_trees.tests.print_assert_details(
        text="'/foo' does not exist",
        expected=True,
        result="/foo" not in Blackboard.storage.keys(),
    )
    assert "/foo" not in Blackboard.storage.keys()
    print("Ticking 'Clear Bar' once...")
    clear_bar.tick_once()
    py_trees.tests.print_assert_details(
        text="'/bar' does not exist",
        expected=True,
        result="/bar" not in Blackboard.storage.keys(),
    )
    assert "/bar" not in Blackboard.storage.keys()
    py_trees.tests.print_assert_details(
        text="'/foo' still does not exist",
        expected=True,
        result="/foo" not in Blackboard.storage.keys(),
    )
    assert "/foo" not in Blackboard.storage.keys()


def test_set_blackboard_variable() -> None:
    console.banner("Set Blackboard Variable")
    blackboard = create_blackboard()
    set_foo = py_trees.behaviours.SetBlackboardVariable(
        name="Set Foo", variable_name="foo", variable_value="bar", overwrite=True
    )
    conservative_set_foo = py_trees.behaviours.SetBlackboardVariable(
        name="Conservative Set Foo",
        variable_name="foo",
        variable_value="bar",
        overwrite=False,
    )

    blackboard.unset("foo")
    py_trees.tests.print_assert_banner()
    set_foo.tick_once()
    py_trees.tests.print_assert_details(
        text="Set 'foo' (doesn't exist)", expected="bar", result=blackboard.foo
    )
    assert "bar" == blackboard.foo
    blackboard.foo = "whoop"
    set_foo.tick_once()
    py_trees.tests.print_assert_details(
        text="Set 'foo' (exists)", expected="bar", result=blackboard.foo
    )
    blackboard.unset("foo")
    py_trees.tests.print_assert_details(
        text="Set 'foo' Status",
        expected=py_trees.common.Status.SUCCESS,
        result=set_foo.status,
    )
    assert set_foo.status == Status.SUCCESS
    conservative_set_foo.tick_once()
    py_trees.tests.print_assert_details(
        text="Conservative 'foo' (doesn't exist)", expected="bar", result=blackboard.foo
    )
    assert "bar" == blackboard.foo
    blackboard.foo = "whoop"
    conservative_set_foo.tick_once()
    py_trees.tests.print_assert_details(
        text="Conservative Set 'foo' (exists)", expected="whoop", result=blackboard.foo
    )
    assert "whoop" == blackboard.foo
    py_trees.tests.print_assert_details(
        text="Conservative Set 'foo' Status",
        expected=py_trees.common.Status.FAILURE,
        result=conservative_set_foo.status,
    )
    assert conservative_set_foo.status == Status.FAILURE

    nested_set_foo = py_trees.behaviours.SetBlackboardVariable(
        name="Nested Set Foo",
        variable_name="nested.foo",
        variable_value="dude",
        overwrite=True,
    )
    nested_set_foo.tick_once()
    py_trees.tests.print_assert_details(
        text="Nested set foo (value)", expected="dude", result=blackboard.nested.foo
    )
    assert blackboard.nested.foo == "dude"
    py_trees.tests.print_assert_details(
        text="Nested set foo (status)",
        expected=py_trees.common.Status.SUCCESS,
        result=nested_set_foo.status,
    )
    assert nested_set_foo.status == py_trees.common.Status.SUCCESS
    blackboard.nested = 5
    nested_set_foo.tick_once()
    py_trees.tests.print_assert_details(
        text="Nested set foo, no nested attribute (status)",
        expected=py_trees.common.Status.FAILURE,
        result=nested_set_foo.status,
    )
    assert nested_set_foo.status == py_trees.common.Status.FAILURE

    class Count:
        value: int = 0

    def generator() -> int:
        Count.value += 1
        return Count.value

    blackboard.unset("foo")
    set_blackboard_variable_from_generator = py_trees.behaviours.SetBlackboardVariable(
        name="Generator Set",
        variable_name="foo",
        variable_value=generator,
        overwrite=True,
    )
    set_blackboard_variable_from_generator.tick_once()
    py_trees.tests.print_assert_details(
        text="Generated Foo", expected=1, result=blackboard.foo
    )
    assert blackboard.foo == 1
    set_blackboard_variable_from_generator.tick_once()
    py_trees.tests.print_assert_details(
        text="Generated Foo", expected=2, result=blackboard.foo
    )
    assert blackboard.foo == 2


def test_check_variable_value() -> None:
    console.banner("Check Variable Value")
    unused_client = create_blackboard()  # noqa: F841 [unused]
    tuples = []
    print(py_trees.display.unicode_blackboard())
    tuples.append(
        (
            py_trees.behaviours.CheckBlackboardVariableValue(
                name="check_foo_equals_bar",
                check=py_trees.common.ComparisonExpression(
                    variable="foo", value="bar", operator=operator.eq
                ),
            ),
            Status.SUCCESS,
        )
    )
    tuples.append(
        (
            py_trees.behaviours.CheckBlackboardVariableValue(
                name="check_foo_equals_foo",
                check=py_trees.common.ComparisonExpression(
                    variable="foo", value="foo", operator=operator.eq
                ),
            ),
            Status.FAILURE,
        )
    )
    tuples.append(
        (
            py_trees.behaviours.CheckBlackboardVariableValue(
                name="check_non_existant_bar_equals_bar",
                check=py_trees.common.ComparisonExpression(
                    variable="bar", value="bar", operator=operator.eq
                ),
            ),
            Status.FAILURE,
        )
    )
    tuples.append(
        (
            py_trees.behaviours.CheckBlackboardVariableValue(
                name="check_nested_foo_equals_bar",
                check=py_trees.common.ComparisonExpression(
                    variable="nested.foo", value="bar", operator=operator.eq
                ),
            ),
            Status.SUCCESS,
        )
    )
    tuples.append(
        (
            py_trees.behaviours.CheckBlackboardVariableValue(
                name="check_nested_foo_equals_foo",
                check=py_trees.common.ComparisonExpression(
                    variable="nested.foo", value="foo", operator=operator.eq
                ),
            ),
            Status.FAILURE,
        )
    )
    for b, unused in tuples:
        b.tick_once()
        print("Feedback message {}".format(b.feedback_message))
    print("")
    py_trees.tests.print_assert_banner()
    for b, asserted_result in tuples:
        py_trees.tests.print_assert_details(
            text=b.name, expected=asserted_result, result=b.status
        )
        assert b.status == asserted_result


def test_check_variable_value_inverted() -> None:
    console.banner("Check Variable Value Neq")
    unused_client = create_blackboard()  # noqa: F841 [unused]
    tuples = []
    print(py_trees.display.unicode_blackboard())
    tuples.append(
        (
            py_trees.behaviours.CheckBlackboardVariableValue(
                name="check_foo_not_equals_bar",
                check=py_trees.common.ComparisonExpression(
                    variable="foo", value="bar", operator=operator.ne
                ),
            ),
            Status.FAILURE,
        )
    )
    tuples.append(
        (
            py_trees.behaviours.CheckBlackboardVariableValue(
                name="check_foo_not_equals_foo",
                check=py_trees.common.ComparisonExpression(
                    variable="foo", value="foo", operator=operator.ne
                ),
            ),
            Status.SUCCESS,
        )
    )
    tuples.append(
        (
            py_trees.behaviours.CheckBlackboardVariableValue(
                name="check_non_existant_bar_not_equals_bar",
                check=py_trees.common.ComparisonExpression(
                    variable="bar", value="bar", operator=operator.ne
                ),
            ),
            Status.FAILURE,
        )
    )
    tuples.append(
        (
            py_trees.behaviours.CheckBlackboardVariableValue(
                name="check_nested_foo_not_equals_bar",
                check=py_trees.common.ComparisonExpression(
                    variable="nested.foo", value="bar", operator=operator.ne
                ),
            ),
            Status.FAILURE,
        )
    )
    tuples.append(
        (
            py_trees.behaviours.CheckBlackboardVariableValue(
                name="check_nested_foo_not_equals_foo",
                check=py_trees.common.ComparisonExpression(
                    variable="nested.foo", value="foo", operator=operator.ne
                ),
            ),
            Status.SUCCESS,
        )
    )
    for b, unused in tuples:
        b.tick_once()
        print("Feedback message {}".format(b.feedback_message))
    print("")
    py_trees.tests.print_assert_banner()
    for b, asserted_result in tuples:
        py_trees.tests.print_assert_details(
            text=b.name, expected=asserted_result, result=b.status
        )
        assert b.status == asserted_result


def test_wait_for_variable_value() -> None:
    console.banner("Wait for Variable Value")
    unused_client = create_blackboard()  # noqa: F841 [unused]
    tuples = []
    print(py_trees.display.unicode_blackboard())
    tuples.append(
        (
            py_trees.behaviours.WaitForBlackboardVariableValue(
                name="check_foo_equals_bar",
                check=py_trees.common.ComparisonExpression(
                    variable="foo", value="bar", operator=operator.eq
                ),
            ),
            Status.SUCCESS,
        )
    )
    tuples.append(
        (
            py_trees.behaviours.WaitForBlackboardVariableValue(
                name="check_foo_equals_foo",
                check=py_trees.common.ComparisonExpression(
                    variable="foo", value="foo", operator=operator.eq
                ),
            ),
            Status.RUNNING,
        )
    )
    tuples.append(
        (
            py_trees.behaviours.WaitForBlackboardVariableValue(
                name="check_non_existant_bar_equals_bar",
                check=py_trees.common.ComparisonExpression(
                    variable="bar", value="bar", operator=operator.eq
                ),
            ),
            Status.RUNNING,
        )
    )
    tuples.append(
        (
            py_trees.behaviours.WaitForBlackboardVariableValue(
                name="check_nested_foo_equals_bar",
                check=py_trees.common.ComparisonExpression(
                    variable="nested.foo", value="bar", operator=operator.eq
                ),
            ),
            Status.SUCCESS,
        )
    )
    tuples.append(
        (
            py_trees.behaviours.WaitForBlackboardVariableValue(
                name="check_nested_foo_equals_foo",
                check=py_trees.common.ComparisonExpression(
                    variable="nested.foo", value="foo", operator=operator.eq
                ),
            ),
            Status.RUNNING,
        )
    )
    for b, unused in tuples:
        b.tick_once()
        print("Feedback message {}".format(b.feedback_message))
    print("")
    py_trees.tests.print_assert_banner()
    for b, asserted_result in tuples:
        py_trees.tests.print_assert_details(
            text=b.name, expected=asserted_result, result=b.status
        )
        assert b.status == asserted_result


def test_check_variable_values() -> None:
    console.banner("Check Variable Values")
    blackboard = Client(name="Blackboard")
    for key in {"a", "b", "c", "d"}:
        blackboard.register_key(key=key, access=py_trees.common.Access.WRITE)
    b = py_trees.behaviours.CheckBlackboardVariableValues(
        name="Checks",
        checks=[
            py_trees.common.ComparisonExpression(
                variable="a", operator=operator.eq, value="a"
            ),
            py_trees.common.ComparisonExpression(
                variable="b", operator=operator.eq, value="b"
            ),
            py_trees.common.ComparisonExpression(
                variable="c", operator=operator.eq, value="c"
            ),
            py_trees.common.ComparisonExpression(
                variable="d", operator=operator.eq, value="d"
            ),
        ],
        operator=operator.and_,
        namespace="results",
    )

    class DataSets(typing.TypedDict):
        a: str
        b: str
        c: str
        d: str
        operator: typing.Callable[[bool, bool], bool]
        text: str
        result: py_trees.common.Status

    datasets: typing.List[DataSets] = [
        {
            "a": "a",
            "b": "b",
            "c": "c",
            "d": "d",
            "operator": operator.and_,
            "text": "AND",
            "result": py_trees.common.Status.SUCCESS,
        },  # noqa: E501 [too-long]
        {
            "a": "z",
            "b": "b",
            "c": "c",
            "d": "d",
            "operator": operator.and_,
            "text": "AND",
            "result": py_trees.common.Status.FAILURE,
        },  # noqa: E501 [too-long]
        {
            "a": "z",
            "b": "b",
            "c": "c",
            "d": "d",
            "operator": operator.or_,
            "text": "OR",
            "result": py_trees.common.Status.SUCCESS,
        },  # noqa: E501 [too-long]
        {
            "a": "z",
            "b": "z",
            "c": "z",
            "d": "z",
            "operator": operator.or_,
            "text": "OR",
            "result": py_trees.common.Status.FAILURE,
        },  # noqa: E501 [too-long]
        {
            "a": "a",
            "b": "z",
            "c": "z",
            "d": "z",
            "operator": operator.xor,
            "text": "XOR",
            "result": py_trees.common.Status.SUCCESS,
        },  # noqa: E501 [too-long]
        {
            "a": "a",
            "b": "b",
            "c": "z",
            "d": "z",
            "operator": operator.xor,
            "text": "XOR",
            "result": py_trees.common.Status.FAILURE,
        },  # noqa: E501 [too-long]
    ]
    print("Comparison Operator: operator.eq")
    print("")
    py_trees.tests.print_assert_banner()
    for data in datasets:
        blackboard.a = data["a"]
        blackboard.b = data["b"]
        blackboard.c = data["c"]
        blackboard.d = data["d"]
        b.operator = data["operator"]
        b.tick_once()
        py_trees.tests.print_assert_details(
            text="[a:{}|b:{}|c:{}|d:{}]{} - {}".format(
                blackboard.a,
                blackboard.b,
                blackboard.c,
                blackboard.d,
                b.feedback_message,
                data["text"],
            ),
            expected=data["result"],
            result=b.status,
        )
        assert b.status == data["result"]


def test_check_blackboard_to_status() -> None:
    console.banner("Check Blackboard to Status")
    blackboard = Client(name="Blackboard")
    blackboard.register_key(key="status", access=py_trees.common.Access.WRITE)
    b = py_trees.behaviours.BlackboardToStatus(
        name="ToStatus",
        variable_name="status",
    )
    for result in {
        py_trees.common.Status.FAILURE,
        py_trees.common.Status.RUNNING,
        py_trees.common.Status.SUCCESS,
    }:
        blackboard.status = result
        b.tick_once()
        py_trees.tests.print_assert_details(
            text=f"ToStatus - {result}", expected=result, result=b.status
        )
        assert b.status == blackboard.status

    blackboard.unset("status")

    with pytest.raises(KeyError) as context:  # if raised, context survives
        print("Unset the blackboard variable - expecting a KeyError")
        b.tick_once()
        py_trees.tests.print_assert_details("KeyError raised", "raised", "not raised")
    py_trees.tests.print_assert_details("KeyError raised", "yes", "yes")
    assert "KeyError" == context.typename
    py_trees.tests.print_assert_details(
        "Substring match", "yet exist", f"{context.value}"
    )
    assert "yet exist" in str(context.value)

    blackboard.status = 5

    with pytest.raises(TypeError) as typeerror_context:  # if raised, context survives
        print("Set a different type - expecting a TypeError")
        b.tick_once()
        py_trees.tests.print_assert_details("TypeError raised", "raised", "not raised")
    py_trees.tests.print_assert_details("TypeError raised", "yes", "yes")
    assert "TypeError" == typeerror_context.typename
    py_trees.tests.print_assert_details(
        "Substring match", "not of type", f"{typeerror_context.value}"
    )
    assert "not of type" in str(typeerror_context.value)

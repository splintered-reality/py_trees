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

##############################################################################
# Logging Level
##############################################################################

# py_trees.logging.level = py_trees.logging.Level.DEBUG
# logger = py_trees.logging.Logger("Nosetest")

##############################################################################
# Helpers
##############################################################################


class FooBar(object):

    def __init__(self):
        self.foo = 'bar'


def create_blackboard():
    """
    Create a blackboard with a few variables.

    Fill with as many different types as we need to get full coverage on
    pretty printing blackboard tests.
    """
    blackboard = py_trees.blackboard.Blackboard(
        name="Tester",
        read={"foo", "some_tuple", "foobar", "nothing"},
        write={"foo", "some_tuple", "foobar", "nothing"}
    )
    blackboard.foo = "bar"
    blackboard.some_tuple = (1, "bar")
    blackboard.foobar = FooBar()
    blackboard.nothing = None
    return blackboard

##############################################################################
# Tests
##############################################################################


def test_variable_exists():
    console.banner("Check Existence of Variable")
    unused_client = create_blackboard()
    tuples = []
    tuples.append((py_trees.blackboard.CheckBlackboardVariable(
        name="check_foo_exists", variable_name="foo"), Status.SUCCESS))
    tuples.append((py_trees.blackboard.CheckBlackboardVariable(
        name="check_bar_exists", variable_name="bar"), Status.FAILURE))
    tuples.append((py_trees.blackboard.CheckBlackboardVariable(
        name="check_nested_foo_exists", variable_name="foobar.foo"), Status.SUCCESS))
    tuples.append((py_trees.blackboard.CheckBlackboardVariable(
        name="check_nested_bar_exists", variable_name="foobar.bar"), Status.FAILURE))
    for b, unused in tuples:
        b.tick_once()
    for b, asserted_result in tuples:
        print("%s: %s [%s]" % (b.name, b.status, asserted_result))
        assert(b.status == asserted_result)


def test_expected_value():
    console.banner("Check Expected Value")
    create_blackboard()
    tuples = []
    tuples.append((py_trees.blackboard.CheckBlackboardVariable(
        name="check_foo_equals_bar", variable_name="foo", expected_value="bar"), Status.SUCCESS))
    tuples.append((py_trees.blackboard.CheckBlackboardVariable(
        name="check_foo_equals_foo", variable_name="foo", expected_value="foo"), Status.FAILURE))
    tuples.append((py_trees.blackboard.CheckBlackboardVariable(
        name="check_bar_equals_bar", variable_name="bar", expected_value="bar"), Status.FAILURE))
    tuples.append((py_trees.blackboard.CheckBlackboardVariable(
        name="check_bar_equals_foo", variable_name="bar", expected_value="foo"), Status.FAILURE))
    tuples.append((py_trees.blackboard.CheckBlackboardVariable(
        name="check_nested_foo_equals_bar", variable_name="foobar.foo", expected_value="bar"),
        Status.SUCCESS))
    tuples.append((py_trees.blackboard.CheckBlackboardVariable(
        name="check_nested_foo_equals_foo", variable_name="foobar.foo", expected_value="foo"),
        Status.FAILURE))
    for b, unused in tuples:
        b.tick_once()
    for b, asserted_result in tuples:
        print("{0}: {1} [{2}]".format(b.name, b.status, asserted_result))
        assert(b.status == asserted_result)


def test_expected_value_inverted():
    console.banner("Check Not Expected Value")
    create_blackboard()

    tuples = []
    tuples.append((py_trees.blackboard.CheckBlackboardVariable(
        name="check_foo_not_equals_bar", variable_name="foo", expected_value="bar",
        comparison_operator=operator.ne), Status.FAILURE))
    tuples.append((py_trees.blackboard.CheckBlackboardVariable(
        name="check_foo_not_equals_foo", variable_name="foo", expected_value="foo",
        comparison_operator=operator.ne), Status.SUCCESS))
    tuples.append((py_trees.blackboard.CheckBlackboardVariable(
        name="check_bar_not_equals_bar", variable_name="bar", expected_value="bar",
        comparison_operator=operator.ne), Status.FAILURE))
    tuples.append((py_trees.blackboard.CheckBlackboardVariable(
        name="check_bar_not_equals_foo", variable_name="bar", expected_value="foo",
        comparison_operator=operator.ne), Status.FAILURE))
    tuples.append((py_trees.blackboard.CheckBlackboardVariable(
        name="check_nested_foo_not_equals_bar", variable_name="foobar.foo", expected_value="bar",
        comparison_operator=operator.ne), Status.FAILURE))
    tuples.append((py_trees.blackboard.CheckBlackboardVariable(
        name="check_nested_foo_not_equals_foo", variable_name="foobar.foo", expected_value="foo",
        comparison_operator=operator.ne), Status.SUCCESS))
    for b, unused in tuples:
        b.tick_once()
    for b, asserted_result in tuples:
        print("%s: %s [%s]" % (b.name, b.status, asserted_result))
        assert(b.status == asserted_result)


def test_wait_for_blackboard_variable():
    console.banner("Wait for Blackboard Variable")
    create_blackboard()

    tuples = []
    tuples.append((py_trees.blackboard.WaitForBlackboardVariable(
        name="check_foo_exists", variable_name="foo"), Status.SUCCESS))
    tuples.append((py_trees.blackboard.WaitForBlackboardVariable(
        name="check_bar_exists", variable_name="bar"), Status.RUNNING))
    tuples.append((py_trees.blackboard.WaitForBlackboardVariable(
        name="check_nested_foo_exists", variable_name="foobar.foo"), Status.SUCCESS))
    tuples.append((py_trees.blackboard.WaitForBlackboardVariable(
        name="check_nested_bar_exists", variable_name="foobar.bar"), Status.RUNNING))
    tuples.append((py_trees.blackboard.WaitForBlackboardVariable(
        name="check_foo_equals_bar", variable_name="foo", expected_value="bar"), Status.SUCCESS))
    tuples.append((py_trees.blackboard.WaitForBlackboardVariable(
        name="check_foo_equals_foo", variable_name="foo", expected_value="foo"), Status.RUNNING))
    tuples.append((py_trees.blackboard.WaitForBlackboardVariable(
        name="check_bar_equals_bar", variable_name="bar", expected_value="bar"), Status.RUNNING))
    tuples.append((py_trees.blackboard.WaitForBlackboardVariable(
        name="check_bar_equals_foo", variable_name="bar", expected_value="foo"), Status.RUNNING))
    tuples.append((py_trees.blackboard.WaitForBlackboardVariable(
        name="check_nested_foo_equals_bar", variable_name="foobar.foo", expected_value="bar"),
        Status.SUCCESS))
    tuples.append((py_trees.blackboard.WaitForBlackboardVariable(
        name="check_nested_foo_equals_foo", variable_name="foobar.foo", expected_value="foo"),
        Status.RUNNING))
    for b, unused in tuples:
        b.tick_once()
    for b, asserted_result in tuples:
        print("%s: %s [%s]" % (b.name, b.status, asserted_result))
        assert(b.status == asserted_result)


def test_clear_blackboard_variable():
    console.banner("Clear Blackboard Variable")
    blackboard = create_blackboard()
    blackboard.foo = "bar"
    clear_foo = py_trees.blackboard.ClearBlackboardVariable(name="Clear Foo", variable_name="foo")
    clear_bar = py_trees.blackboard.ClearBlackboardVariable(name="Clear Bar", variable_name="bar")
    print("%s" % blackboard)
    print(" - Assert blackboard has a value for key 'foo'")
    assert("foo" in py_trees.blackboard.Blackboard.storage.keys())
    print(" - Clearing 'foo'")
    clear_foo.tick_once()
    print(" - Assert blackboard does not have attribute 'foo'")
    assert("foo" not in py_trees.blackboard.Blackboard.storage.keys())
    print(" - Clearing 'bar'")
    clear_bar.tick_once()
    print(" - Asserting nothing wierd happened")
    assert("foo" not in py_trees.blackboard.Blackboard.storage.keys())


def test_set_blackboard_variable():
    console.banner("Set Blackboard Variable")
    blackboard = create_blackboard()
    set_foo = py_trees.blackboard.SetBlackboardVariable(
        name="Set Foo", variable_name="foo", variable_value="bar")
    print(" - Set 'foo'")
    set_foo.tick_once()
    print("\n%s" % blackboard)
    print(" - Assert blackboard.foo='bar'")
    assert(hasattr(blackboard, "foo"))
    assert(blackboard.foo == "bar")
    print(" - Assert set_foo.status == SUCCESS")
    assert(set_foo.status == Status.SUCCESS)

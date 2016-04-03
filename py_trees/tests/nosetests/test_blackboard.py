#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/gopher_crazy_hospital/py_trees/license/LICENSE
#

##############################################################################
# Imports
##############################################################################

# enable some python3 compatibility options:
# (unicode_literals not compatible with python2 uuid module)
from __future__ import absolute_import, print_function

from nose.tools import assert_raises

import py_trees
from py_trees import Blackboard, CheckBlackboardVariable, Status
import rocon_console.console as console
import operator

##############################################################################
# Logging Level
##############################################################################

# py_trees.logging.level = py_trees.logging.Level.DEBUG
# logger = py_trees.logging.get_logger("Nosetest")

##############################################################################
# Helpers
##############################################################################

def create_blackboard():
    blackboard = Blackboard()
    blackboard.foo = "bar"
    return blackboard

##############################################################################
# Tests
##############################################################################

def test_print_blackboard():
    print(console.bold + "\n****************************************************************************************" + console.reset)
    print(console.bold + "* Blackboard" + console.reset)
    print(console.bold + "****************************************************************************************\n" + console.reset)
    blackboard = create_blackboard()
    print("%s" % blackboard)


def test_variable_exists():
    print(console.bold + "\n****************************************************************************************" + console.reset)
    print(console.bold + "* Check Existence of Variable" + console.reset)
    print(console.bold + "****************************************************************************************\n" + console.reset)
    blackboard = create_blackboard()
    tuples = []
    tuples.append((CheckBlackboardVariable(name="check_foo_exists", variable_name="foo"), Status.SUCCESS))
    tuples.append((CheckBlackboardVariable(name="check_bar_exists", variable_name="bar"), Status.FAILURE))
    for b, unused in tuples:
        b.tick_once()
    for b, asserted_result in tuples:
        print("%s: %s [%s]" % (b.name, b.status, asserted_result))
        assert(b.status == asserted_result)


def test_expected_value():
    print(console.bold + "\n****************************************************************************************" + console.reset)
    print(console.bold + "* Check Expected Value" + console.reset)
    print(console.bold + "****************************************************************************************\n" + console.reset)
    blackboard = create_blackboard()
    tuples = []
    tuples.append((CheckBlackboardVariable(name="check_foo_equals_bar", variable_name="foo", expected_value="bar"), Status.SUCCESS))
    tuples.append((CheckBlackboardVariable(name="check_foo_equals_foo", variable_name="foo", expected_value="foo"), Status.FAILURE))
    tuples.append((CheckBlackboardVariable(name="check_bar_equals_bar", variable_name="bar", expected_value="bar"), Status.FAILURE))
    tuples.append((CheckBlackboardVariable(name="check_bar_equals_foo", variable_name="bar", expected_value="foo"), Status.FAILURE))
    for b, unused in tuples:
        b.tick_once()
    for b, asserted_result in tuples:
        print("%s: %s [%s]" % (b.name, b.status, asserted_result))
        assert(b.status == asserted_result)

def test_expected_value_inverted():
    print(console.bold + "\n****************************************************************************************" + console.reset)
    print(console.bold + "* Check Not Expected Value" + console.reset)
    print(console.bold + "****************************************************************************************\n" + console.reset)
    blackboard = create_blackboard()

    tuples = []
    tuples.append((CheckBlackboardVariable(name="check_foo_equals_bar", variable_name="foo", expected_value="bar", comparison_operator=operator.ne), Status.FAILURE))
    tuples.append((CheckBlackboardVariable(name="check_foo_equals_foo", variable_name="foo", expected_value="foo", comparison_operator=operator.ne), Status.SUCCESS))
    tuples.append((CheckBlackboardVariable(name="check_bar_equals_bar", variable_name="bar", expected_value="bar", comparison_operator=operator.ne), Status.FAILURE))
    tuples.append((CheckBlackboardVariable(name="check_bar_equals_foo", variable_name="bar", expected_value="foo", comparison_operator=operator.ne), Status.FAILURE))
    for b, unused in tuples:
        b.tick_once()
    for b, asserted_result in tuples:
        print("%s: %s [%s]" % (b.name, b.status, asserted_result))
        assert(b.status == asserted_result)

def test_clear_blackboard_variable():
    print(console.bold + "\n****************************************************************************************" + console.reset)
    print(console.bold + "* Clear Blackboard Variable" + console.reset)
    print(console.bold + "****************************************************************************************\n" + console.reset)
    blackboard = create_blackboard()
    blackboard.foo = "bar"
    clear_foo = py_trees.blackboard.ClearBlackboardVariable(name="Clear Foo", variable_name="foo")
    clear_bar = py_trees.blackboard.ClearBlackboardVariable(name="Clear Bar", variable_name="bar")
    print("%s" % blackboard)
    print(" - Assert blackboard has attribute 'foo'")
    assert(hasattr(blackboard, "foo"))
    print(" - Clearing 'foo'")
    clear_foo.tick_once()
    print(" - Assert blackboard does not have attribute 'foo'")
    assert(not hasattr(blackboard, "foo"))
    print(" - Clearing 'bar'")
    clear_bar.tick_once()
    print(" - Asserting nothing wierd happened")
    assert(not hasattr(blackboard, "foo"))

def test_set_blackboard_variable():
    print(console.bold + "\n****************************************************************************************" + console.reset)
    print(console.bold + "* Set Blackboard Variable" + console.reset)
    print(console.bold + "****************************************************************************************\n" + console.reset)
    blackboard = create_blackboard()
    set_foo = py_trees.blackboard.SetBlackboardVariable(name="Set Foo", variable_name="foo", variable_value="bar")
    print(" - Set 'foo'")
    set_foo.tick_once()
    print("\n%s" % blackboard)
    print(" - Assert blackboard.foo='bar'")
    assert(hasattr(blackboard, "foo"))
    assert(blackboard.foo == "bar")
    print(" - Assert set_foo.status == SUCCESS")
    assert(set_foo.status == py_trees.Status.SUCCESS)

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


def test_behaviour_from_function_naming() -> None:
    console.banner("Test Behaviour From Function Naming")

    def foo() -> py_trees.common.Status:
        return py_trees.common.Status.SUCCESS

    foo_instance = py_trees.meta.create_behaviour_from_function(foo)(name="Foo")
    success = py_trees.behaviours.Success(name="Success")
    named_success = py_trees.meta.create_behaviour_from_function(
        py_trees.behaviours.success
    )(name="Woohoo")

    print("\n--------- Assertions ---------\n")
    print("foo_instance.name = {} [Foo]".format(foo_instance.name))
    assert foo_instance.name == "Foo"
    print("success.name = {}".format(success.name))
    assert success.name == "Success"
    print("named_success.name == {} Woohoo".format(named_success.name))
    assert named_success.name == "Woohoo"

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
import py_trees.utilities as utilities

##############################################################################
# Tests
##############################################################################


def test_valid_filenames() -> None:
    console.banner("Valid Filenames")
    names = {
        "With\nNewlines": "with_newlines",
        "With Spaces": "with_spaces",
        " Leading Space": "leading_space",
        "Trailing Space ": "trailing_space",
    }
    print(
        console.green
        + "------------------ Assertions ------------------\n"
        + console.reset
    )
    for name, expected_name in names.items():
        print(
            console.cyan
            + repr(name)
            + ": "
            + console.yellow
            + expected_name
            + " ["
            + utilities.get_valid_filename(name)
            + "]"
            + console.reset
        )
        assert utilities.get_valid_filename(name) == expected_name


def test_get_fully_qualified_name() -> None:
    console.banner("Fully Qualified Names")
    pairs = {
        "py_trees.behaviours.Periodic": py_trees.behaviours.Periodic(name="foo", n=3),
        "py_trees.decorators.Inverter": py_trees.decorators.Inverter(
            name="Inverter", child=py_trees.behaviours.Success(name="Success")
        ),
        "py_trees.behaviours.Success": py_trees.behaviours.Success(name="Success"),
    }
    print(
        console.green
        + "------------------ Assertions ------------------\n"
        + console.reset
    )
    for expected_name, object_instance in pairs.items():
        print(
            console.cyan
            + expected_name
            + console.white
            + " == "
            + console.yellow
            + utilities.get_fully_qualified_name(object_instance)
            + console.reset
        )
        assert expected_name == utilities.get_fully_qualified_name(object_instance)

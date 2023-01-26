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
import py_trees.tests
import pytest

##############################################################################
# Logging Level
##############################################################################

py_trees.logging.level = py_trees.logging.Level.DEBUG
logger = py_trees.logging.Logger("Tests")

##############################################################################
# Tests
##############################################################################


def test_timer_invalid_duration() -> None:
    console.banner("Timer Exceptions - Invalid Duration")
    with pytest.raises(TypeError) as context:  # if raised, context survives
        # intentional error -> silence mypy
        unused_timer = py_trees.timers.Timer(  # noqa: F841 [unused]
            name="Timer", duration="invalid_type"  # type: ignore[arg-type]
        )
        py_trees.tests.print_assert_details("TypeError raised", "raised", "not raised")
    py_trees.tests.print_assert_details("TypeError raised", "yes", "yes")
    assert "TypeError" == context.typename
    py_trees.tests.print_assert_details(
        "  substring match", "duration", f"{context.value}"
    )
    assert "duration" in str(context.value)

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


def test_probabilistic_behaviour_workflow() -> None:
    console.banner("Probabilistic Behaviour")

    with pytest.raises(ValueError) as context:  # if raised, context survives
        # intentional error -> silence mypy
        unused_root = py_trees.behaviours.ProbabilisticBehaviour(  # noqa: F841 [unused]
            name="ProbabilisticBehaviour", weights="invalid_type"  # type: ignore[arg-type]
        )
        py_trees.tests.print_assert_details("ValueError raised", "raised", "not raised")
    py_trees.tests.print_assert_details("ValueError raised", "yes", "yes")
    assert "ValueError" == context.typename

    root = py_trees.behaviours.ProbabilisticBehaviour(
        name="ProbabilisticBehaviour", weights=[0.0, 0.0, 1.0]
    )

    py_trees.tests.print_assert_details(
        text="task not yet ticked",
        expected=py_trees.common.Status.INVALID,
        result=root.status,
    )
    assert root.status == py_trees.common.Status.INVALID

    root.tick_once()
    py_trees.tests.print_assert_details(
        text="task ticked once",
        expected=py_trees.common.Status.RUNNING,
        result=root.status,
    )
    assert root.status == py_trees.common.Status.RUNNING

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
# Eternal Guard
##############################################################################


def test_eventually() -> None:
    console.banner("Eventually")

    blackboard = py_trees.blackboard.Client()
    blackboard.register_key(key="flag", access=py_trees.common.Access.WRITE)
    blackboard.flag = False

    set_flag_true = py_trees.behaviours.SetBlackboardVariable(
        name="SetFlag", variable_name="flag", variable_value=True, overwrite=True
    )
    worker = py_trees.behaviours.TickCounter(
        name="Counter-1", duration=1, completion_status=py_trees.common.Status.SUCCESS
    )
    parallel = py_trees.idioms.eventually(
        name="Parallel", worker=worker, on_completion=set_flag_true
    )
    parallel.tick_once()
    py_trees.tests.print_assert_banner()
    py_trees.tests.print_assert_details(
        text="BB Variable (flag)",
        expected=False,
        result=blackboard.flag,
    )
    print(py_trees.display.unicode_tree(root=parallel, show_status=True))
    assert not blackboard.flag

    parallel.tick_once()
    py_trees.tests.print_assert_banner()
    py_trees.tests.print_assert_details(
        text="BB Variable (flag)",
        expected=True,
        result=blackboard.flag,
    )
    print(py_trees.display.unicode_tree(root=parallel, show_status=True))
    assert blackboard.flag


def test_eventually_swiss() -> None:
    console.banner("Eventually (Swiss)")
    set_result_true = py_trees.behaviours.SetBlackboardVariable(
        name="SetResultTrue",
        variable_name="result",
        variable_value=True,
        overwrite=True,
    )
    set_result_false = py_trees.behaviours.SetBlackboardVariable(
        name="SetResultFalse",
        variable_name="result",
        variable_value=False,
        overwrite=True,
    )
    worker = py_trees.behaviours.TickCounter(
        name="Counter", duration=1, completion_status=py_trees.common.Status.FAILURE
    )
    root = py_trees.idioms.eventually_swiss(
        name="Count with Result",
        workers=[worker],
        on_failure=set_result_false,
        on_success=set_result_true,
    )
    blackboard = py_trees.blackboard.Client()
    blackboard.register_key(key="result", access=py_trees.common.Access.READ)

    root.tick_once()
    print(py_trees.display.unicode_tree(root=root, show_status=True))
    print(py_trees.display.unicode_blackboard())

    py_trees.tests.print_assert_banner()
    with pytest.raises(KeyError) as context:  # if raised, context survives
        print("With blackboard.get('result') ...")
        print("Expecting a KeyError with substring 'does not yet exist'\n")
        blackboard.get("result")
    # py_trees.tests.print_assert_details("KeyError raised", "raised", "not raised")
    py_trees.tests.print_assert_details("KeyError raised", "yes", "yes")
    assert "KeyError" == context.typename
    py_trees.tests.print_assert_details(
        "  substring match", "does not yet exist", f"{context.value}"
    )
    assert "does not yet exist" in str(context.value)

    root.tick_once()
    print(py_trees.display.unicode_tree(root=root, show_status=True))
    print(py_trees.display.unicode_blackboard())

    py_trees.tests.print_assert_banner()
    py_trees.tests.print_assert_details(
        text="BB Variable (flag)",
        expected=False,
        result=blackboard.result,
    )
    assert not blackboard.result

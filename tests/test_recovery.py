# tests/test_recovery.py

from py_trees.behaviours import StatusQueue
from py_trees.common import Status
from py_trees.composites import Recovery


def test_main_success() -> None:
    main = StatusQueue("Main", [Status.SUCCESS], eventually=None)
    root = Recovery("Recovery", children=[main])

    root.tick_once()
    assert main.status == Status.SUCCESS


def test_main_running() -> None:
    main = StatusQueue("Main", [Status.RUNNING, Status.SUCCESS], eventually=None)
    root = Recovery("Recovery", children=[main])

    root.tick_once()
    assert root.status == Status.RUNNING

    root.tick_once()
    assert root.status == Status.SUCCESS


def test_recovery_success_then_retry_main() -> None:
    # main fails, recovery1 succeeds, main succeeds when retried
    main = StatusQueue("Main", [Status.FAILURE, Status.SUCCESS], eventually=None)
    rec1 = StatusQueue("Rec1", [Status.SUCCESS], eventually=None)
    root = Recovery("Recovery", children=[main, rec1])

    # tick 1: main fails, recovery1 succeeds, composite RUNNING
    root.tick_once()
    assert root.status == Status.RUNNING

    # tick 2: main retried, succeeds
    root.tick_once()
    assert root.status == Status.SUCCESS


def test_recovery_fails_then_next_succeeds() -> None:
    # main fails, rec1 fails, rec2 succeeds, then main succeeds
    main = StatusQueue("Main", [Status.FAILURE, Status.SUCCESS], eventually=None)
    rec1 = StatusQueue("Rec1", [Status.FAILURE], eventually=None)
    rec2 = StatusQueue("Rec2", [Status.SUCCESS], eventually=None)
    root = Recovery("Recovery", children=[main, rec1, rec2])

    # tick 1: main fails, rec1 fails, composite RUNNING
    root.tick_once()
    assert root.status == Status.RUNNING

    # tick 2: main retried, succeeds
    root.tick_once()
    assert root.status == Status.SUCCESS


def test_all_recoveries_fail() -> None:
    # main fails, all recoveries fail, composite fails
    main = StatusQueue("Main", [Status.FAILURE], eventually=None)
    rec1 = StatusQueue("Rec1", [Status.FAILURE], eventually=None)
    rec2 = StatusQueue("Rec2", [Status.FAILURE], eventually=None)
    root = Recovery("Recovery", children=[main, rec1, rec2])

    root.tick_once()
    assert root.status == Status.FAILURE

#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/splintered-reality/py_trees/devel/LICENSE
#

##############################################################################
# Imports
##############################################################################

import itertools

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


def test_parallel_failure() -> None:
    console.banner("Parallel Failure")
    success_on_one = py_trees.common.ParallelPolicy.SuccessOnOne()
    success_on_all_synchronised = py_trees.common.ParallelPolicy.SuccessOnAll(
        synchronise=True
    )
    success_on_all_not_synchronised = py_trees.common.ParallelPolicy.SuccessOnAll(
        synchronise=False
    )
    for policy in [
        success_on_one,
        success_on_all_synchronised,
        success_on_all_not_synchronised,
    ]:
        root = py_trees.composites.Parallel(name="Parallel", policy=policy)
        failure = py_trees.behaviours.Failure("Failure")
        success = py_trees.behaviours.Success("Success")
        root.add_child(failure)
        root.add_child(success)
        print(py_trees.display.unicode_tree(root))
        visitor = py_trees.visitors.DebugVisitor()
        py_trees.tests.tick_tree(root, 1, 1, visitors=[visitor])

        print("\n--------- Assertions ---------\n")
        print("root.status == py_trees.common.Status.FAILURE")
        assert root.status == py_trees.common.Status.FAILURE
        print("failure.status == py_trees.common.Status.FAILURE")
        assert failure.status == py_trees.common.Status.FAILURE
        print("success.status == py_trees.common.Status.SUCCESS")
        assert success.status == py_trees.common.Status.SUCCESS

    failure = py_trees.behaviours.Failure("Failure")
    success = py_trees.behaviours.Success("Success")
    for child, synchronise in itertools.product([success, failure], [True, False]):
        policy = py_trees.common.ParallelPolicy.SuccessOnSelected(
            children=[child], synchronise=synchronise
        )
        root = py_trees.composites.Parallel(name="Parallel", policy=policy)
        root.add_child(failure)
        root.add_child(success)
        print(py_trees.display.unicode_tree(root))
        visitor = py_trees.visitors.DebugVisitor()
        py_trees.tests.tick_tree(root, 1, 1, visitors=[visitor])

        print("\n--------- Assertions ---------\n")
        print("root.status == py_trees.common.Status.FAILURE")
        assert root.status == py_trees.common.Status.FAILURE
        print("failure.status == py_trees.common.Status.FAILURE")
        assert failure.status == py_trees.common.Status.FAILURE
        print("success.status == py_trees.common.Status.SUCCESS")
        assert success.status == py_trees.common.Status.SUCCESS

        # cleanup so they can be added again
        root.remove_child(failure)
        root.remove_child(success)


def test_parallel_success() -> None:
    console.banner("Parallel Success")
    root = py_trees.composites.Parallel(
        name="Parallel", policy=py_trees.common.ParallelPolicy.SuccessOnAll()
    )
    success1 = py_trees.behaviours.Success("Success1")
    success2 = py_trees.behaviours.Success("Success2")
    root.add_child(success1)
    root.add_child(success2)
    print(py_trees.display.unicode_tree(root))
    visitor = py_trees.visitors.DebugVisitor()
    py_trees.tests.tick_tree(root, 1, 1, visitors=[visitor])

    print("\n--------- Assertions ---------\n")
    print("root.status == py_trees.common.Status.SUCCESS")
    assert root.status == py_trees.common.Status.SUCCESS
    print("success1.status == py_trees.common.Status.SUCCESS")
    assert success1.status == py_trees.common.Status.SUCCESS
    print("success2.status == py_trees.common.Status.SUCCESS")
    assert success2.status == py_trees.common.Status.SUCCESS


def test_parallel_running() -> None:
    console.banner("Parallel Running")
    root = py_trees.composites.Parallel(
        name="Parallel", policy=py_trees.common.ParallelPolicy.SuccessOnAll()
    )
    success_after_1 = py_trees.behaviours.StatusQueue(
        name="SuccessAfter1",
        queue=[py_trees.common.Status.RUNNING],
        eventually=py_trees.common.Status.SUCCESS,
    )
    running = py_trees.behaviours.Running("Running")
    success_every_other = py_trees.behaviours.SuccessEveryN("SuccessEveryOther", 2)
    root.add_child(success_after_1)
    root.add_child(running)
    root.add_child(success_every_other)
    print(py_trees.display.unicode_tree(root))
    visitor = py_trees.visitors.DebugVisitor()
    py_trees.tests.tick_tree(root, 1, 1, visitors=[visitor])

    print("\n--------- Assertions ---------\n")
    print("root.status == py_trees.common.Status.FAILURE")
    assert root.status == py_trees.common.Status.FAILURE
    print("success_after_1.status == py_trees.common.Status.INVALID")
    assert success_after_1.status == py_trees.common.Status.INVALID
    print("running.status == py_trees.common.Status.INVALID")
    assert running.status == py_trees.common.Status.INVALID
    print("success_every_other.status == py_trees.common.Status.FAILURE")
    assert success_every_other.status == py_trees.common.Status.FAILURE

    py_trees.tests.tick_tree(root, 2, 2, visitors=[visitor])

    print("\n--------- Assertions ---------\n")
    print("root.status == py_trees.common.Status.RUNNING")
    assert root.status == py_trees.common.Status.RUNNING
    print("success_after_1.status == py_trees.common.Status.SUCCESS")
    assert success_after_1.status == py_trees.common.Status.SUCCESS
    print("running.status == py_trees.common.Status.RUNNING")
    assert running.status == py_trees.common.Status.RUNNING
    print("success_every_other.status == py_trees.common.Status.SUCCESS")
    assert success_every_other.status == py_trees.common.Status.SUCCESS


def test_parallel_success_on_one() -> None:
    console.banner("Parallel Success on One")
    print("")
    root = py_trees.composites.Parallel(
        name="Parallel", policy=py_trees.common.ParallelPolicy.SuccessOnOne()
    )
    running1 = py_trees.behaviours.Running("Running1")
    success = py_trees.behaviours.Success("Success")
    running2 = py_trees.behaviours.Running("Running2")
    root.add_child(running1)
    root.add_child(success)
    root.add_child(running2)
    print(py_trees.display.unicode_tree(root))
    visitor = py_trees.visitors.DebugVisitor()
    py_trees.tests.tick_tree(root, 1, 1, visitors=[visitor], print_snapshot=True)

    print("\n--------- Assertions ---------\n")
    print("root.status == py_trees.common.Status.SUCCESS")
    assert root.status == py_trees.common.Status.SUCCESS
    print("running1.status == py_trees.common.Status.INVALID")
    assert running1.status == py_trees.common.Status.INVALID
    print("success.status == py_trees.common.Status.SUCCESS")
    assert success.status == py_trees.common.Status.SUCCESS
    print("running2.status == py_trees.common.Status.INVALID")
    assert running2.status == py_trees.common.Status.INVALID


def test_parallel_success_on_selected() -> None:
    console.banner("Parallel Success on Selected")
    print("")
    running1 = py_trees.behaviours.Running(name="Running1")
    success1 = py_trees.behaviours.StatusQueue(
        name="Success1",
        queue=[py_trees.common.Status.RUNNING],
        eventually=py_trees.common.Status.SUCCESS,
    )
    success2 = py_trees.behaviours.StatusQueue(
        name="Success1",
        queue=[py_trees.common.Status.RUNNING],
        eventually=py_trees.common.Status.SUCCESS,
    )
    running2 = py_trees.behaviours.Running(name="Running2")

    root = py_trees.composites.Parallel(
        name="Parallel",
        policy=py_trees.common.ParallelPolicy.SuccessOnSelected(
            children=[success1, success2]
        ),
    )
    root.add_children([running1, success1, success2, running2])

    print(py_trees.display.unicode_tree(root))
    visitor = py_trees.visitors.DebugVisitor()

    py_trees.tests.tick_tree(root, 1, 1, visitors=[visitor], print_snapshot=True)
    print("\n--------- Assertions ---------\n")
    print("root.status == py_trees.common.Status.RUNNING")
    assert root.status == py_trees.common.Status.RUNNING
    print("running1.status == py_trees.common.Status.RUNNING")
    assert running1.status == py_trees.common.Status.RUNNING
    print("success1.status == py_trees.common.Status.RUNNING")
    assert success1.status == py_trees.common.Status.RUNNING
    print("success2.status == py_trees.common.Status.RUNNING")
    assert success2.status == py_trees.common.Status.RUNNING
    print("running2.status == py_trees.common.Status.RUNNING")
    assert running2.status == py_trees.common.Status.RUNNING

    py_trees.tests.tick_tree(root, 2, 3, visitors=[visitor], print_snapshot=True)
    print("\n--------- Assertions ---------\n")
    print("root.status == py_trees.common.Status.SUCCESS")
    assert root.status == py_trees.common.Status.SUCCESS
    print("running1.status == py_trees.common.Status.INVALID")
    assert running1.status == py_trees.common.Status.INVALID
    print("success1.status == py_trees.common.Status.SUCCESS")
    assert success1.status == py_trees.common.Status.SUCCESS
    print("success2.status == py_trees.common.Status.SUCCESS")
    assert success2.status == py_trees.common.Status.SUCCESS
    print("running2.status == py_trees.common.Status.INVALID")
    assert running2.status == py_trees.common.Status.INVALID


def test_parallel_success_on_selected_invalid_configuration() -> None:
    console.banner("Parallel Success on Selected [Invalid Configuration]")
    print("")
    running1 = py_trees.behaviours.Running(name="Running1")
    running2 = py_trees.behaviours.Running(name="Running2")
    running3 = py_trees.behaviours.Running(name="Running3")

    different_policy = py_trees.common.ParallelPolicy.SuccessOnSelected(
        children=[running1, running2]
    )
    empty_policy = py_trees.common.ParallelPolicy.SuccessOnSelected(children=[])
    parallel = py_trees.composites.Parallel(
        name="Parallel",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
        children=[running1, running3],
    )
    for policy in [different_policy, empty_policy]:
        print("")
        print(
            console.cyan
            + "Policy Children: "
            + console.yellow
            + str([c.name for c in policy.children])
            + console.reset
        )
        print(
            console.cyan
            + "Parallel Children: "
            + console.yellow
            + str([c.name for c in parallel.children])
            + console.reset
        )
        parallel.policy = policy
        print("\n--------- Assertions ---------\n")
        print("setup() raises a 'RuntimeError' due to invalid configuration")

        with pytest.raises(RuntimeError) as context:  # if raised, context survives
            parallel.setup()
            py_trees.tests.print_assert_details(
                "RuntimeError raised", "raised", "not raised"
            )
        py_trees.tests.print_assert_details("RuntimeError raised", "yes", "yes")
        assert "RuntimeError" == context.typename
        py_trees.tests.print_assert_details(
            "Substring match", "SuccessOnSelected", f"{context.value}"
        )
        assert "SuccessOnSelected" in str(context.value)

        print("initialise() raises a 'RuntimeError' due to invalid configuration")

        with pytest.raises(RuntimeError) as context:  # if raised, context survives
            parallel.tick_once()
            py_trees.tests.print_assert_details(
                "RuntimeError raised", "raised", "not raised"
            )
        py_trees.tests.print_assert_details("RuntimeError raised", "yes", "yes")
        assert "RuntimeError" == context.typename
        py_trees.tests.print_assert_details(
            "Substring match", "SuccessOnSelected", f"{context.value}"
        )
        assert "SuccessOnSelected" in str(context.value)


def test_parallel_synchronisation() -> None:
    console.banner("Parallel Synchronisation")
    root = py_trees.composites.Parallel(
        name="Parallel",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=True),
    )
    success = py_trees.behaviours.Success(name="Success")
    success_every_second = py_trees.decorators.FailureIsRunning(
        name="SuccessEverySecond",
        child=py_trees.behaviours.SuccessEveryN(name="Flipper", n=2),
    )

    root.add_children([success, success_every_second])
    print(py_trees.display.unicode_tree(root))
    debug_visitor = py_trees.visitors.DebugVisitor()
    snapshot_visitor = py_trees.visitors.SnapshotVisitor()
    py_trees.tests.tick_tree(
        root, 1, 1, visitors=[debug_visitor, snapshot_visitor], print_snapshot=True
    )

    print("\n--------- Assertions ---------\n")
    print("root.status == py_trees.common.Status.RUNNING")
    assert root.status == py_trees.common.Status.RUNNING
    print("success.status == py_trees.common.Status.SUCCESS")
    assert success.status == py_trees.common.Status.SUCCESS
    print("success_every_second.status == py_trees.common.Status.RUNNING")
    assert success_every_second.status == py_trees.common.Status.RUNNING

    snapshot_visitor.initialise()
    py_trees.tests.tick_tree(
        root, 2, 2, visitors=[debug_visitor, snapshot_visitor], print_snapshot=True
    )

    print("\n--------- Assertions ---------\n")
    print("root.status == py_trees.common.Status.SUCCESS")
    assert root.status == py_trees.common.Status.SUCCESS
    print("success.status == py_trees.common.Status.SUCCESS")
    assert success.status == py_trees.common.Status.SUCCESS
    print("success_every_second.status == py_trees.common.Status.SUCCESS")
    assert success_every_second.status == py_trees.common.Status.SUCCESS
    print(
        "success [id: {}] did not get ticked [snapshot: {}]".format(
            success.id, [str(ident) for ident in snapshot_visitor.visited.keys()]
        )
    )
    assert success.id not in snapshot_visitor.visited

    snapshot_visitor.initialise()
    py_trees.tests.tick_tree(
        root, 3, 3, visitors=[debug_visitor, snapshot_visitor], print_snapshot=True
    )

    print("\n--------- Assertions ---------\n")
    print("root.status == py_trees.common.Status.RUNNING")
    assert root.status == py_trees.common.Status.RUNNING
    print("success.status == py_trees.common.Status.SUCCESS")
    assert success.status == py_trees.common.Status.SUCCESS
    print("success_every_second.status == py_trees.common.Status.RUNNING")
    assert success_every_second.status == py_trees.common.Status.RUNNING


def test_parallel_no_synchronisation() -> None:
    console.banner("Parallel No Synchronisation")
    root = py_trees.composites.Parallel(
        name="Parallel",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=False),
    )
    success_every_two = py_trees.decorators.FailureIsRunning(
        name="SuccessEverySecond",
        child=py_trees.behaviours.SuccessEveryN(name="Flipper2", n=2),
    )
    success_every_three = py_trees.decorators.FailureIsRunning(
        name="SuccessEveryThird",
        child=py_trees.behaviours.SuccessEveryN(name="Flipper3", n=3),
    )

    debug_visitor = py_trees.visitors.DebugVisitor()
    snapshot_visitor = py_trees.visitors.SnapshotVisitor()

    root.add_children([success_every_two, success_every_three])
    for counter in range(1, 6):
        snapshot_visitor.initialise()
        py_trees.tests.tick_tree(
            root,
            counter,
            counter,
            visitors=[debug_visitor, snapshot_visitor],
            print_snapshot=True,
        )
        print(counter)
        if counter % 2 == 0:
            print("success_every_two.status == py_trees.common.Status.SUCCESS")
            assert success_every_two.status == py_trees.common.Status.SUCCESS
        elif counter % 3 == 0:
            print("success_every_three.status == py_trees.common.Status.SUCCESS")
            assert success_every_three.status == py_trees.common.Status.SUCCESS
        else:
            print("success_every_two.status == py_trees.common.Status.RUNNING")
            assert success_every_two.status == py_trees.common.Status.RUNNING
            print("success_every_three.status == py_trees.common.Status.RUNNING")
            assert success_every_three.status == py_trees.common.Status.RUNNING
        print("root.status == py_trees.common.Status.RUNNING")
        assert root.status == py_trees.common.Status.RUNNING

    snapshot_visitor.initialise()
    py_trees.tests.tick_tree(root, 6, 6, print_snapshot=True)
    print("success_every_two.status == py_trees.common.Status.SUCCESS")
    assert success_every_two.status == py_trees.common.Status.SUCCESS
    print("success_every_three.status == py_trees.common.Status.SUCCESS")
    assert success_every_three.status == py_trees.common.Status.SUCCESS
    print("root.status == py_trees.common.Status.SUCCESS")
    assert root.status == py_trees.common.Status.SUCCESS


def test_add_tick_remove_with_current_child() -> None:
    console.banner("Add (Failure)-Tick-Remove with Current Child")
    py_trees.tests.print_assert_banner()
    root = py_trees.composites.Parallel(
        name="Parallel",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=False),
    )
    child = py_trees.behaviours.Failure(name="Failure")
    root.add_child(child)
    root.tick_once()
    print(py_trees.display.unicode_tree(root, show_status=True))
    root.remove_child(child)
    print(py_trees.display.unicode_tree(root, show_status=True))
    py_trees.tests.print_assert_details("Current Child", None, root.current_child)
    assert root.current_child is None

    root.add_child(child)
    root.tick_once()
    print(py_trees.display.unicode_tree(root, show_status=True))
    root.remove_all_children()
    print(py_trees.display.unicode_tree(root, show_status=True))
    py_trees.tests.print_assert_details("Current Child", None, root.current_child)
    assert root.current_child is None

    replacement = py_trees.behaviours.Success(name="Replacement")
    root.add_child(child)
    root.tick_once()
    print(py_trees.display.unicode_tree(root, show_status=True))
    root.replace_child(child=child, replacement=replacement)
    print(py_trees.display.unicode_tree(root, show_status=True))
    py_trees.tests.print_assert_details("Current Child", None, root.current_child)
    assert root.current_child is None

    root.remove_all_children()
    root.add_child(child)
    root.tick_once()
    print(py_trees.display.unicode_tree(root, show_status=True))
    root.remove_child_by_id(child_id=child.id)
    print(py_trees.display.unicode_tree(root, show_status=True))
    py_trees.tests.print_assert_details("Current Child", None, root.current_child)
    assert root.current_child is None


def test_tick_add_with_current_child() -> None:
    console.banner("Tick-Add with Current Child")
    py_trees.tests.print_assert_banner()
    root = py_trees.composites.Parallel(
        name="Parallel",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=False),
    )
    root.tick_once()
    child = py_trees.behaviours.Failure(name="Failure")
    root.add_child(child)
    py_trees.tests.print_assert_details("Current Child", None, root.current_child)
    assert root.current_child is None


def test_add_tick_add_with_current_child() -> None:
    console.banner("Add-Tick-Add with Current Child")
    py_trees.tests.print_assert_banner()
    root = py_trees.composites.Parallel(
        name="Parallel",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=False),
    )
    run1 = py_trees.behaviours.Running("Run1")
    run2 = py_trees.behaviours.Running("Run2")
    root.add_child(run1)
    root.tick_once()
    print(py_trees.display.unicode_tree(root, show_status=True))
    assert root.current_child is not None
    py_trees.tests.print_assert_details(
        "Current Child", run1.name, root.current_child.name
    )
    assert root.current_child.name == run1.name
    root.add_child(run2)
    print(py_trees.display.unicode_tree(root, show_status=True))
    assert root.current_child is not None
    py_trees.tests.print_assert_details(
        "Current Child", run1.name, root.current_child.name
    )
    assert root.current_child.name == run1.name


def test_add_tick_insert_with_current_child() -> None:
    console.banner("Add-Tick-Insert with Current Child")
    py_trees.tests.print_assert_banner()
    root = py_trees.composites.Parallel(
        name="Parallel",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=False),
    )
    run1 = py_trees.behaviours.Running("Run1")
    run2 = py_trees.behaviours.Running("Run2")
    root.add_child(run1)
    root.tick_once()
    print(py_trees.display.unicode_tree(root, show_status=True))
    assert root.current_child is not None
    py_trees.tests.print_assert_details(
        "Current Child", run1.name, root.current_child.name
    )
    assert root.current_child.name == run1.name
    root.insert_child(run2, index=0)
    print(py_trees.display.unicode_tree(root, show_status=True))
    assert root.current_child is not None
    py_trees.tests.print_assert_details(
        "Current Child", run1.name, root.current_child.name
    )
    assert root.current_child.name == run1.name

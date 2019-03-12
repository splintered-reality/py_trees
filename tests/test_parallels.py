#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/splintered-reality/py_trees/devel/LICENSE
#

##############################################################################
# Imports
##############################################################################

import nose.tools
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


def test_parallel_failure():
    console.banner("Parallel Failure")
    root = py_trees.composites.Parallel("Parallel")
    failure = py_trees.behaviours.Failure("Failure")
    success = py_trees.behaviours.Success("Success")
    root.add_child(failure)
    root.add_child(success)
    print(py_trees.display.ascii_tree(root))
    visitor = py_trees.visitors.DebugVisitor()
    py_trees.tests.tick_tree(root, 1, 1, visitors=[visitor])

    print("\n--------- Assertions ---------\n")
    print("root.status == py_trees.common.Status.FAILURE")
    assert(root.status == py_trees.common.Status.FAILURE)
    print("failure.status == py_trees.common.Status.FAILURE")
    assert(failure.status == py_trees.common.Status.FAILURE)
    print("success.status == py_trees.common.Status.SUCCESS")
    assert(success.status == py_trees.common.Status.SUCCESS)


def test_parallel_success():
    console.banner("Parallel Success")
    root = py_trees.composites.Parallel("Parallel")
    success1 = py_trees.behaviours.Success("Success1")
    success2 = py_trees.behaviours.Success("Success2")
    root.add_child(success1)
    root.add_child(success2)
    print(py_trees.display.ascii_tree(root))
    visitor = py_trees.visitors.DebugVisitor()
    py_trees.tests.tick_tree(root, 1, 1, visitors=[visitor])

    print("\n--------- Assertions ---------\n")
    print("root.status == py_trees.common.Status.SUCCESS")
    assert(root.status == py_trees.common.Status.SUCCESS)
    print("success1.status == py_trees.common.Status.SUCCESS")
    assert(success1.status == py_trees.common.Status.SUCCESS)
    print("success2.status == py_trees.common.Status.SUCCESS")
    assert(success2.status == py_trees.common.Status.SUCCESS)


def test_parallel_running():
    console.banner("Parallel Running")
    root = py_trees.composites.Parallel(
        "Parallel", policy=py_trees.common.ParallelPolicy.SuccessOnAll())
    success_after_1 = py_trees.behaviours.Count(
        name="SuccessAfter1",
        fail_until=0,
        running_until=1,
        success_until=20,
        reset=False)

    running = py_trees.behaviours.Running("Running")
    success_every_other = py_trees.behaviours.SuccessEveryN("SuccessEveryOther", 2)
    root.add_child(success_after_1)
    root.add_child(running)
    root.add_child(success_every_other)
    print(py_trees.display.ascii_tree(root))
    visitor = py_trees.visitors.DebugVisitor()
    py_trees.tests.tick_tree(root, 1, 1, visitors=[visitor])

    print("\n--------- Assertions ---------\n")
    print("root.status == py_trees.common.Status.FAILURE")
    assert(root.status == py_trees.common.Status.FAILURE)
    print("success_after_1.status == py_trees.common.Status.INVALID")
    assert(success_after_1.status == py_trees.common.Status.INVALID)
    print("running.status == py_trees.common.Status.INVALID")
    assert(running.status == py_trees.common.Status.INVALID)
    print("success_every_other.status == py_trees.common.Status.FAILURE")
    assert(success_every_other.status == py_trees.common.Status.FAILURE)

    py_trees.tests.tick_tree(root, 2, 2, visitors=[visitor])

    print("\n--------- Assertions ---------\n")
    print("root.status == py_trees.common.Status.RUNNING")
    assert(root.status == py_trees.common.Status.RUNNING)
    print("success_after_1.status == py_trees.common.Status.SUCCESS")
    assert(success_after_1.status == py_trees.common.Status.SUCCESS)
    print("running.status == py_trees.common.Status.RUNNING")
    assert(running.status == py_trees.common.Status.RUNNING)
    print("success_every_other.status == py_trees.common.Status.SUCCESS")
    assert(success_every_other.status == py_trees.common.Status.SUCCESS)


def test_parallel_success_on_one():
    console.banner("Parallel Success on One")
    print("")
    root = py_trees.composites.Parallel(
        name="Parallel", policy=py_trees.common.ParallelPolicy.SuccessOnOne())
    running1 = py_trees.behaviours.Running("Running1")
    success = py_trees.behaviours.Success("Success")
    running2 = py_trees.behaviours.Running("Running2")
    root.add_child(running1)
    root.add_child(success)
    root.add_child(running2)
    print(py_trees.display.ascii_tree(root))
    visitor = py_trees.visitors.DebugVisitor()
    py_trees.tests.tick_tree(root, 1, 1, visitors=[visitor], print_snapshot=True)

    print("\n--------- Assertions ---------\n")
    print("root.status == py_trees.common.Status.SUCCESS")
    assert(root.status == py_trees.common.Status.SUCCESS)
    print("running1.status == py_trees.common.Status.INVALID")
    assert(running1.status == py_trees.common.Status.INVALID)
    print("success.status == py_trees.common.Status.SUCCESS")
    assert(success.status == py_trees.common.Status.SUCCESS)
    print("running2.status == py_trees.common.Status.INVALID")
    assert(running2.status == py_trees.common.Status.INVALID)


def test_parallel_success_on_selected():
    console.banner("Parallel Success on Selected")
    print("")
    running1 = py_trees.behaviours.Running(name="Running1")
    success1 = py_trees.behaviours.Count(
        name="Success1",
        fail_until=0,
        running_until=1,
        success_until=10)
    success2 = py_trees.behaviours.Count(
        name="Success2",
        fail_until=0,
        running_until=2,
        success_until=10)
    running2 = py_trees.behaviours.Running(name="Running2")

    root = py_trees.composites.Parallel(
        name="Parallel",
        policy=py_trees.common.ParallelPolicy.SuccessOnSelected(
            children=[success1, success2])
        )
    root.add_children([running1, success1, success2, running2])

    print(py_trees.display.ascii_tree(root))
    visitor = py_trees.visitors.DebugVisitor()

    py_trees.tests.tick_tree(root, 1, 1, visitors=[visitor], print_snapshot=True)
    print("\n--------- Assertions ---------\n")
    print("root.status == py_trees.common.Status.RUNNING")
    assert(root.status == py_trees.common.Status.RUNNING)
    print("running1.status == py_trees.common.Status.RUNNING")
    assert(running1.status == py_trees.common.Status.RUNNING)
    print("success1.status == py_trees.common.Status.RUNNING")
    assert(success1.status == py_trees.common.Status.RUNNING)
    print("success2.status == py_trees.common.Status.RUNNING")
    assert(success2.status == py_trees.common.Status.RUNNING)
    print("running2.status == py_trees.common.Status.RUNNING")
    assert(running2.status == py_trees.common.Status.RUNNING)

    py_trees.tests.tick_tree(root, 2, 3, visitors=[visitor], print_snapshot=True)
    print("\n--------- Assertions ---------\n")
    print("root.status == py_trees.common.Status.SUCCESS")
    assert(root.status == py_trees.common.Status.SUCCESS)
    print("running1.status == py_trees.common.Status.INVALID")
    assert(running1.status == py_trees.common.Status.INVALID)
    print("success1.status == py_trees.common.Status.SUCCESS")
    assert(success1.status == py_trees.common.Status.SUCCESS)
    print("success2.status == py_trees.common.Status.SUCCESS")
    assert(success2.status == py_trees.common.Status.SUCCESS)
    print("running2.status == py_trees.common.Status.INVALID")
    assert(running2.status == py_trees.common.Status.INVALID)


def test_parallel_success_on_selected_invalid_configuration():
    console.banner("Parallel Success on Selected [Invalid Configuration]")
    print("")
    running1 = py_trees.behaviours.Running(name="Running1")
    running2 = py_trees.behaviours.Running(name="Running2")
    running3 = py_trees.behaviours.Running(name="Running3")

    different_policy = py_trees.common.ParallelPolicy.SuccessOnSelected(
            children=[running1, running2]
    )
    empty_policy = py_trees.common.ParallelPolicy.SuccessOnSelected(
            children=[]
    )
    parallel = py_trees.composites.Parallel(
        name="Parallel",
        children=[running1, running3]
    )
    for policy in [different_policy, empty_policy]:
        print("")
        print(console.cyan + "Policy Children: " + console.yellow + str([c.name for c in policy.children]) + console.reset)
        print(console.cyan + "Parallel Children: " + console.yellow + str([c.name for c in parallel.children]) + console.reset)
        parallel.policy = policy
        print("\n--------- Assertions ---------\n")
        print("setup() raises a 'RuntimeError' due to invalid configuration")
        with nose.tools.assert_raises(RuntimeError) as context:
            parallel.setup()
            print("RuntimeError has message with substring 'SuccessOnSelected'")
            assert("SuccessOnSelected" in str(context.exception))
        print("initialise() raises a 'RuntimeError' due to invalid configuration")
        with nose.tools.assert_raises(RuntimeError) as context:
            parallel.tick_once()
            print("RuntimeError has message with substring 'SuccessOnSelected'")
            assert("SuccessOnSelected" in str(context.exception))


def test_parallel_synchronisation():
    console.banner("Parallel Synchronisation")
    root = py_trees.composites.Parallel(
        name="Parallel",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(
            synchronise=True
        ))
    success = py_trees.behaviours.Success()
    success_every_second = py_trees.decorators.FailureIsRunning(
        name="SuccessEverySecond",
        child=py_trees.behaviours.SuccessEveryN(name="Flipper", n=2)
    )

    root.add_children([success, success_every_second])
    print(py_trees.display.ascii_tree(root))
    debug_visitor = py_trees.visitors.DebugVisitor()
    snapshot_visitor = py_trees.visitors.SnapshotVisitor()
    py_trees.tests.tick_tree(
        root, 1, 1,
        visitors=[debug_visitor, snapshot_visitor],
        print_snapshot=True
    )

    print("\n--------- Assertions ---------\n")
    print("root.status == py_trees.common.Status.RUNNING")
    assert(root.status == py_trees.common.Status.RUNNING)
    print("success.status == py_trees.common.Status.SUCCESS")
    assert(success.status == py_trees.common.Status.SUCCESS)
    print("success_every_second.status == py_trees.common.Status.RUNNING")
    assert(success_every_second.status == py_trees.common.Status.RUNNING)

    snapshot_visitor.initialise()
    py_trees.tests.tick_tree(
        root, 2, 2,
        visitors=[debug_visitor, snapshot_visitor],
        print_snapshot=True
    )

    print("\n--------- Assertions ---------\n")
    print("root.status == py_trees.common.Status.SUCCESS")
    assert(root.status == py_trees.common.Status.SUCCESS)
    print("success.status == py_trees.common.Status.SUCCESS")
    assert(success.status == py_trees.common.Status.SUCCESS)
    print("success_every_second.status == py_trees.common.Status.SUCCESS")
    assert(success_every_second.status == py_trees.common.Status.SUCCESS)
    print("success [id: {}] did not get ticked [snapshot: {}]".format(success.id, [str(ident) for ident in snapshot_visitor.visited.keys()]))
    assert(success.id not in snapshot_visitor.visited)

    snapshot_visitor.initialise()
    py_trees.tests.tick_tree(
        root, 3, 3,
        visitors=[debug_visitor, snapshot_visitor],
        print_snapshot=True
    )

    print("\n--------- Assertions ---------\n")
    print("root.status == py_trees.common.Status.RUNNING")
    assert(root.status == py_trees.common.Status.RUNNING)
    print("success.status == py_trees.common.Status.SUCCESS")
    assert(success.status == py_trees.common.Status.SUCCESS)
    print("success_every_second.status == py_trees.common.Status.RUNNING")
    assert(success_every_second.status == py_trees.common.Status.RUNNING)

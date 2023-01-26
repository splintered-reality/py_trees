#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/splintered-reality/py_trees/devel/LICENSE
#
##############################################################################
# Imports
##############################################################################

# enable some python3 compatibility options:
# (unicode_literals not compatible with python2 uuid module)
from __future__ import absolute_import, print_function

import py_trees
import py_trees.console as console

##############################################################################
# Logging Level
##############################################################################

py_trees.logging.level = py_trees.logging.Level.DEBUG
logger = py_trees.logging.Logger("Nosetest")

##############################################################################
# Subtrees
##############################################################################


def test_oneshot_with_fail_causes_reentry() -> None:
    def decorator_oneshot(
        child: py_trees.behaviour.Behaviour, policy: py_trees.common.OneShotPolicy
    ) -> py_trees.decorators.OneShot:
        return py_trees.decorators.OneShot(name="OneShot", child=child, policy=policy)

    def idiom_oneshot(
        child: py_trees.behaviour.Behaviour, policy: py_trees.common.OneShotPolicy
    ) -> py_trees.behaviour.Behaviour:
        return py_trees.idioms.oneshot(
            name="Oneshot Idiom",
            variable_name="oneshot",
            behaviour=child,
            policy=policy,
        )

    for title, create_tree in [
        ("Idiom", idiom_oneshot),
        ("Decorator", decorator_oneshot),
    ]:
        for policy in py_trees.common.OneShotPolicy:
            # Setup
            console.banner(
                "{} w/ Failure Causes Reentry [policy: {}]".format(title, policy.name)
            )
            py_trees.tests.clear_blackboard()

            # Tree
            fail_then_run = py_trees.behaviours.StatusQueue(
                name="Fail Then Run",
                queue=[
                    py_trees.common.Status.FAILURE,
                    py_trees.common.Status.RUNNING,
                    py_trees.common.Status.RUNNING,
                ],
                eventually=py_trees.common.Status.FAILURE,
            )
            oneshot = create_tree(fail_then_run, policy)

            # Ticking
            py_trees.tests.tick_tree(
                root=oneshot,
                from_tick=1,
                to_tick=1,
                print_snapshot=True,
                print_blackboard=True,
            )
            print(console.green + "Asserts (After Failure)" + console.reset)
            print(
                console.cyan
                + "  OneShot Status: "
                + console.yellow
                + "{}".format(oneshot.status)
                + console.reset
                + " [{}]".format(py_trees.common.Status.FAILURE)
            )
            print(
                console.cyan
                + "  Child Status  : "
                + console.yellow
                + "{}".format(fail_then_run.status)
                + console.reset
                + " [{}]".format(py_trees.common.Status.FAILURE)
            )
            assert oneshot.status == py_trees.common.Status.FAILURE
            assert fail_then_run.status == py_trees.common.Status.FAILURE

            # Ticking
            py_trees.tests.tick_tree(
                root=oneshot,
                from_tick=2,
                to_tick=2,
                print_snapshot=True,
                print_blackboard=True,
            )
            expected_oneshot_status = {
                py_trees.common.OneShotPolicy.ON_COMPLETION: py_trees.common.Status.FAILURE,
                py_trees.common.OneShotPolicy.ON_SUCCESSFUL_COMPLETION: py_trees.common.Status.RUNNING,
            }
            expected_behaviour_status = {
                "Idiom": {
                    py_trees.common.OneShotPolicy.ON_COMPLETION: py_trees.common.Status.INVALID,
                    py_trees.common.OneShotPolicy.ON_SUCCESSFUL_COMPLETION: py_trees.common.Status.RUNNING,
                },
                "Decorator": {
                    py_trees.common.OneShotPolicy.ON_COMPLETION: py_trees.common.Status.FAILURE,
                    py_trees.common.OneShotPolicy.ON_SUCCESSFUL_COMPLETION: py_trees.common.Status.RUNNING,
                },
            }
            print(console.green + "Asserts (After Running)" + console.reset)
            print(
                console.cyan
                + "  OneShot Status: "
                + console.yellow
                + "{}".format(oneshot.status)
                + console.reset
                + " [{}]".format(expected_oneshot_status[policy])
            )
            print(
                console.cyan
                + "  Child Status  : "
                + console.yellow
                + "{}".format(fail_then_run.status)
                + console.reset
                + " [{}]".format(expected_behaviour_status[title][policy])
            )
            assert oneshot.status == expected_oneshot_status[policy]
            assert fail_then_run.status == expected_behaviour_status[title][policy]


def untest_oneshot_with_subtrees_and_interrupt() -> None:
    sequence_subtree = py_trees.composites.Sequence(
        name="Sequence",
        memory=True,
        children=[
            py_trees.behaviours.Success(name="Success 1"),
            py_trees.behaviours.Success(name="Success 2"),
        ],
    )
    selector_subtree = py_trees.composites.Selector(
        name="Selector",
        memory=False,
        children=[
            py_trees.behaviours.Failure(name="Failure"),
            py_trees.behaviours.Success(name="Success"),
        ],
    )
    for worker_subtree in [sequence_subtree, selector_subtree]:
        decorator_oneshot = py_trees.decorators.OneShot(
            name="OneShot",
            child=worker_subtree,
            policy=py_trees.common.OneShotPolicy.ON_SUCCESSFUL_COMPLETION,
        )
        idiom_oneshot = py_trees.idioms.oneshot(
            name="OneShot", variable_name="oneshot", behaviour=worker_subtree
        )
        for title, oneshot in [
            ("Idiom", idiom_oneshot),
            ("Decorator", decorator_oneshot),
        ]:
            console.banner("{} w/ {} and Interrupt".format(title, worker_subtree.name))
            py_trees.tests.clear_blackboard()

            # Tree with higher priority branch
            root = py_trees.composites.Selector(name="Root", memory=False)
            high_priority = py_trees.behaviours.StatusQueue(
                name="High Priority",
                queue=[
                    py_trees.common.Status.FAILURE,
                    py_trees.common.Status.SUCCESS,
                ],
                eventually=None,
            )
            root.add_children([high_priority, oneshot])

            # Ticking
            py_trees.tests.tick_tree(
                root=root,
                from_tick=1,
                to_tick=1,
                print_snapshot=True,
                print_blackboard=True,
            )
            py_trees.tests.tick_tree(
                root=root,
                from_tick=2,
                to_tick=2,
                print_snapshot=True,
                print_blackboard=True,
            )
            print(console.green + "Asserts (After Interrupt)" + console.reset)
            print(
                console.cyan
                + "  OneShot Status: "
                + console.yellow
                + "{}".format(oneshot.status)
                + console.reset
                + " [{}]".format(py_trees.common.Status.INVALID)
            )
            print(
                console.cyan
                + "  Child Status  : "
                + console.yellow
                + "{}".format(worker_subtree.status)
                + console.reset
                + " [{}]".format(py_trees.common.Status.INVALID)
            )
            assert oneshot.status == py_trees.common.Status.INVALID
            assert worker_subtree.status == py_trees.common.Status.INVALID
            py_trees.tests.tick_tree(
                root=root,
                from_tick=3,
                to_tick=3,
                print_snapshot=True,
                print_blackboard=True,
            )
            print(console.green + "Asserts (After Re-entry)" + console.reset)
            print(
                console.cyan
                + "  OneShot Status: "
                + console.yellow
                + "{}".format(oneshot.status)
                + console.reset
                + " [{}]".format(py_trees.common.Status.SUCCESS)
            )
            print(
                console.cyan
                + "  Child Status  : "
                + console.yellow
                + "{}".format(worker_subtree.status)
                + console.reset
                + " [{}]".format(py_trees.common.Status.INVALID)
            )
            assert oneshot.status == py_trees.common.Status.SUCCESS
            assert worker_subtree.status == py_trees.common.Status.INVALID

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
# Mirror
##############################################################################


def test_mirror():
    console.banner("Mirror")

    root = py_trees.composites.Parallel(
        name="Root",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=False)
    )
    original = py_trees.behaviours.Count(
        name="High Priority",
        fail_until=1,
        running_until=2,
        success_until=3,
        reset=False
    )
    mirror = py_trees.behaviours.Mirror(
        name="Mirror",
        mirrored=original
    )
    root.add_children([original, mirror])
    py_trees.tests.print_assert_banner()
    for index in range(1, 4):
        root.tick_once()
        py_trees.tests.print_assert_details(
            text="Tick {}.............".format(index),
            expected=original.status,
            result=mirror.status
        )
        assert(original.status == mirror.status)
    root.stop(py_trees.common.Status.INVALID)
    py_trees.tests.print_assert_details(
        text="Priority Interrupt..".format(index),
        expected=original.status,
        result=mirror.status
    )
    assert(original.status == mirror.status)

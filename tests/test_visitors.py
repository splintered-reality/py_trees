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
# Classes
##############################################################################

##############################################################################
# Tests
##############################################################################


def test_winds_of_change():
    console.banner("Winds of Change")

    root = py_trees.composites.Selector(name='Selector')
    a = py_trees.behaviours.Count(name="A")
    b = py_trees.behaviours.Count(name="B")
    c = py_trees.behaviours.Count(name="C", fail_until=0, running_until=3, success_until=15)
    root.add_child(a)
    root.add_child(b)
    root.add_child(c)
    print(py_trees.display.ascii_tree(root))

    debug_visitor = py_trees.visitors.DebugVisitor()
    snapshot_visitor = py_trees.visitors.SnapshotVisitor()
    winds_of_change_visitor = py_trees.visitors.WindsOfChangeVisitor()

    for i, result in zip(range(1, 5), [True, False, False, True]):
        py_trees.tests.tick_tree(
            root, i, i,
            visitors=[debug_visitor,
                      snapshot_visitor,
                      winds_of_change_visitor],
            print_snapshot=True
        )
        print("--------- Assertions ---------\n")
        print("winds_of_change_visitor.changed == {}".format(result))
        assert(winds_of_change_visitor.changed is result)

    print("Done")

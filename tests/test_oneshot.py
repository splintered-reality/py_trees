#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/stonier/py_trees/devel/LICENSE
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
# Helpers
##############################################################################


def tick_tree(root, from_iteration, to_iteration):
    """
    Tick the tree and return the node count on the last tick.
    @returns count : the number of nodes traversed on the last tick
    """
    print("\n================== Iteration %s-%s ==================\n" % (from_iteration, to_iteration))
    for i in range(from_iteration, to_iteration + 1):
        count = 0
        print(("\n--------- Run %s ---------\n" % i))
        for unused_node in root.tick():
            count += 1
    return count


##############################################################################
# Tests
##############################################################################

def test_oneshot_does_not_modify_class():
    print(console.bold + "\n****************************************************************************************" + console.reset)
    print(console.bold + "* Test Oneshots Do Not Modify the Original Class" + console.reset)
    print(console.bold + "****************************************************************************************\n" + console.reset)
    oneshot = py_trees.meta.oneshot(py_trees.composites.Sequence)(name="Oneshot",
                                                                  children=[py_trees.behaviours.Failure(name="Failure")])
    normal = py_trees.composites.Sequence(name="Normal",
                                          children=[py_trees.behaviours.Failure(name="Failure")])
    oneshot_count = tick_tree(oneshot, 1, 2)
    print(console.yellow + "\nOneshot Sequence Tick Count: {}".format(oneshot_count) + console.reset)
    normal_count = tick_tree(normal, 1, 2)
    print(console.yellow + "\nNormal Sequence Tick Count: {}".format(normal_count) + console.reset)
    assert(oneshot_count == 1)
    # Hitherto, the decorator modified the sequence permanently, so the tick count for the latter
    # would be the same. This behaviour is confusing - the decorators should always create
    # *new* classes so there is only one policy for decoration - instance modification only.
    assert(normal_count == 2)

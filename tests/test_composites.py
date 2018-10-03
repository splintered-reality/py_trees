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
# Tests
##############################################################################


def test_replacing_children():
    console.banner("Replacing Children")
    parent = py_trees.composites.Sequence(name="Parent")
    old_child = py_trees.behaviours.Success(name="Old Child")
    new_child = py_trees.behaviours.Success(name="New Child")
    parent.add_child(old_child)
    parent.replace_child(old_child, new_child)
    print("\n--------- Assertions ---------\n")
    # Disabled until #108 is fixed
    # print("old_child.parent == None")
    # assert(old_child.parent == None)
    print("new_child.parent is parent")
    assert(new_child.parent is parent)


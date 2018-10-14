#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/stonier/py_trees/devel/LICENSE
#

##############################################################################
# Imports
##############################################################################

import py_trees
import py_trees.console as console
import py_trees.display as display

##############################################################################
# Tests
##############################################################################


def test_symbols():
    console.banner("Symbols")
    # This test has no assertions, except from the human eye. When it fails,
    # ticks and crosses fail and must be inspected by the human eye
    print("[{0}][{1}][{2}][{3}]".format(
        display._behaviour_status_to_ascii[py_trees.common.Status.SUCCESS],
        display._behaviour_status_to_ascii[py_trees.common.Status.FAILURE],
        display._behaviour_status_to_ascii[py_trees.common.Status.INVALID],
        display._behaviour_status_to_ascii[py_trees.common.Status.RUNNING]))

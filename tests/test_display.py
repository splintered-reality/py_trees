#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/stonier/py_trees/devel/LICENSE
#

##############################################################################
# Imports
##############################################################################

# python2-3 compatibility - either this, or prefix with a u for python 2 below
#     print(u"[{0}][{1}][{2}][{3}]".format(
from __future__ import unicode_literals

import py_trees
import py_trees.console as console
import py_trees.display as display

##############################################################################
# Tests
##############################################################################


def test_symbols():
    print(console.bold + "\n****************************************************************************************" + console.reset)
    print(console.bold + "* Symbols" + console.reset)
    print(console.bold + "****************************************************************************************" + console.reset)
    # This test has no assertions, except from the human eye. When it fails,
    # ticks and crosses fail and must be inspected by the human eye
    print("[{0}][{1}][{2}][{3}]".format(
        display.ascii_check_mark(py_trees.common.Status.SUCCESS),
        display.ascii_check_mark(py_trees.common.Status.FAILURE),
        display.ascii_check_mark(py_trees.common.Status.INVALID),
        display.ascii_check_mark(py_trees.common.Status.RUNNING)))

    print("[{0}][{1}][{2}][{3}]".format(
        display.ascii_bullet(py_trees.behaviours.Success()),
        display.ascii_bullet(py_trees.composites.Sequence()),
        display.ascii_bullet(py_trees.composites.Selector()),
        display.ascii_bullet(py_trees.composites.Parallel())))

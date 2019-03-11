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
import py_trees.display as display

##############################################################################
# Tests
##############################################################################


def test_symbols():
    console.banner("Symbols")
    # This test has no assertions, except from the human eye. When it fails,
    # ticks and crosses fail and must be inspected by the human eye
    if console.has_unicode():
        symbols = display.unicode_symbols
    else:
        symbols = display.ascii_symbols
    print("[{0}][{1}][{2}][{3}]".format(
        symbols[py_trees.common.Status.SUCCESS],
        symbols[py_trees.common.Status.FAILURE],
        symbols[py_trees.common.Status.INVALID],
        symbols[py_trees.common.Status.RUNNING]
        )
    )

    print("[{0}][{1}][{2}][{3}]".format(
        symbols[py_trees.behaviours.Success()],
        symbols[py_trees.composites.Sequence()],
        symbols[py_trees.composites.Selector()],
        symbols[py_trees.composites.Parallel()]
        )
    )

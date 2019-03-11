#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/splintered-reality/py_trees/devel/LICENSE
#

##############################################################################
# Imports
##############################################################################

import py_trees


##############################################################################
# Tests
##############################################################################


def test_correct_encode():
    assert py_trees.console.define_symbol_or_fallback(u'\u26A1', "a", "ascii") == "a"
    assert py_trees.console.define_symbol_or_fallback(u'\u26A1', "a", "utf-8") == u'\u26A1'

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


def test_timer_errors():
    console.banner("Timer Errors")
    print("__init__ raises a 'TypeError' due to invalid duration being passed")
    with nose.tools.assert_raises(TypeError) as context:
        unused_timer = py_trees.timers.Timer(name="Timer", duration="invalid_type")
        print("TypeError has message with substring 'duration'")
        assert("duration" in str(context.exception))

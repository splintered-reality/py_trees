#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/gopher_crazy_hospital/py_trees/license/LICENSE
#

##############################################################################
# Imports
##############################################################################

# enable some python3 compatibility options:
# (unicode_literals not compatible with python2 uuid module)
from __future__ import absolute_import, print_function

from nose.tools import assert_raises
import gopher_semantics
import rocon_console.console as console

##############################################################################
# Globals
##############################################################################

class SharedData:
    __semantics = None

    def __init__(self):
        if SharedData.__semantics is None:
            self.semantics = gopher_semantics.Semantics()
    
##############################################################################
# Tests
##############################################################################


def test_semantics():
    print(console.bold + "\n****************************************************************************************" + console.reset)
    print(console.bold + "* Selector" + console.reset)
    print(console.bold + "****************************************************************************************" + console.reset)
    assert(True)

 


# def test_foo():
#     print('--------- Nosetest Logs ---------')
#     print_logging()
#     py_trees.foo1()
#     py_trees.foo2()
#     print('--------- Behaviour Logs ---------')
#     d = py_trees.behaviours.Count(name="D")
#     d.initialise()
#     print("Done")
#     assert(True)

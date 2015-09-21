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
import gopher_behaviours
import gopher_semantics
import gopher_semantic_msgs.msg as gopher_semantic_msgs
import os
import rocon_console.console as console

##############################################################################
# Globals
##############################################################################

semantics = gopher_semantics.load_desirable_destinations()

##############################################################################
# Tests
##############################################################################


def test_topological_paths():
    print(console.bold + "\n****************************************************************************************" + console.reset)
    print(console.bold + "* Topological Paths - Downward Spiral to Hell " + console.reset)
    print(console.bold + "****************************************************************************************" + console.reset)
    locations = ["sofa_in_front_of_tv", "pizza_shop", "ashokas_hell"]
    topological_path = gopher_behaviours.planner.find_topological_path("heaven", locations, semantics)
    print(console.green + "\nTopological Path:" + console.reset)
    for node in topological_path:
        if isinstance(node, gopher_semantic_msgs.Elevator):
            print(console.cyan + "  Elevator" + console.reset + ": " + console.yellow + node.unique_name + console.reset)
        elif isinstance(node, gopher_semantic_msgs.Location):
            print(console.cyan + "  Location" + console.reset + ": " + console.yellow + node.unique_name + console.reset)
    assert(isinstance(topological_path[0], gopher_semantic_msgs.Location))
    assert(isinstance(topological_path[1], gopher_semantic_msgs.Location))
    assert(isinstance(topological_path[2], gopher_semantic_msgs.Elevator))
    assert(isinstance(topological_path[3], gopher_semantic_msgs.Location))

    print(console.bold + "\n****************************************************************************************" + console.reset)
    print(console.bold + "* Topological Paths - To Hell And Back" + console.reset)
    print(console.bold + "****************************************************************************************" + console.reset)
    locations = ["ashokas_hell", "sofa_in_front_of_tv", "pizza_shop"]
    topological_path = gopher_behaviours.planner.find_topological_path("heaven", locations, semantics)
    print(console.green + "\nTopological Path:" + console.reset)
    for node in topological_path:
        if isinstance(node, gopher_semantic_msgs.Elevator):
            print(console.cyan + "  Elevator" + console.reset + ": " + console.yellow + node.unique_name + console.reset)
        elif isinstance(node, gopher_semantic_msgs.Location):
            print(console.cyan + "  Location" + console.reset + ": " + console.yellow + node.unique_name + console.reset)
    assert(isinstance(topological_path[0], gopher_semantic_msgs.Elevator))
    assert(isinstance(topological_path[1], gopher_semantic_msgs.Location))
    assert(isinstance(topological_path[2], gopher_semantic_msgs.Elevator))
    assert(isinstance(topological_path[3], gopher_semantic_msgs.Location))
    assert(isinstance(topological_path[4], gopher_semantic_msgs.Location))

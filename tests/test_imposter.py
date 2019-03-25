#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/splintered-reality/py_trees/devel/LICENSE
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

def create_impostered_composite():
    return py_trees.meta.failure_is_running(py_trees.composites.Sequence)("Impostered Composite")


def create_impostered_behaviour():
    return py_trees.meta.success_is_failure(py_trees.behaviours.Success)("Impostered Behaviour")


def has_child_with_name(parent, child_name):
    return child_name if child_name in [c.name for c in parent.children] else None


##############################################################################
# Tests
##############################################################################

def test_imposter_has_add_child_method():
    print(console.bold + "\n****************************************************************************************" + console.reset)
    print(console.bold + "* Test Imposter has add_child_method" + console.reset)
    print(console.bold + "****************************************************************************************\n" + console.reset)
    tuples = []
    tuples.append((create_impostered_behaviour(), False))
    tuples.append((create_impostered_composite(), True))
    for b, asserted_result in tuples:
        print("%s has add_child: %s [%s]" % (b.name, hasattr(b, 'add_child'), asserted_result))
        assert(hasattr(b, 'add_child') == asserted_result)


def test_parent_chain():
    print(console.bold + "\n****************************************************************************************" + console.reset)
    print(console.bold + "* Test Parent Chain" + console.reset)
    print(console.bold + "****************************************************************************************\n" + console.reset)
    root = py_trees.composites.Parallel("Root")
    sequence_failure_is_running = create_impostered_composite()
    success_is_failure = create_impostered_behaviour()

    sequence_failure_is_running.add_child(success_is_failure)
    root.add_child(sequence_failure_is_running)

    tuples = []
    tuples.append((success_is_failure, sequence_failure_is_running.name))
    tuples.append((sequence_failure_is_running, root.name))
    for child, asserted_result in tuples:
        print("%s's parent: %s [%s]" % (child.name, child.parent.name, asserted_result))
        assert(child.parent.name == asserted_result)


def test_parent_chain_with_add_children():
    print(console.bold + "\n****************************************************************************************" + console.reset)
    print(console.bold + "* Test Parent Chain with add_children" + console.reset)
    print(console.bold + "****************************************************************************************\n" + console.reset)
    root = py_trees.composites.Parallel("Root")
    sequence_failure_is_running = create_impostered_composite()
    success_is_failure = create_impostered_behaviour()

    sequence_failure_is_running.add_children([success_is_failure])
    root.add_children([sequence_failure_is_running])

    tuples = []
    tuples.append((success_is_failure, sequence_failure_is_running.name))
    tuples.append((sequence_failure_is_running, root.name))
    for child, asserted_result in tuples:
        print("%s's parent: %s [%s]" % (child.name, child.parent.name, asserted_result))
        assert(child.parent.name == asserted_result)


def test_child_chain():
    print(console.bold + "\n****************************************************************************************" + console.reset)
    print(console.bold + "* Test Child Chain" + console.reset)
    print(console.bold + "****************************************************************************************\n" + console.reset)
    root = py_trees.composites.Parallel("Root")
    sequence_failure_is_running = create_impostered_composite()
    success_is_failure = create_impostered_behaviour()

    sequence_failure_is_running.add_child(success_is_failure)
    root.add_child(sequence_failure_is_running)

    tuples = []
    tuples.append((root, sequence_failure_is_running.name))
    tuples.append((sequence_failure_is_running, success_is_failure.name))
    for parent, asserted_result in tuples:
        print("%s's child: %s [%s]" % (parent.name, has_child_with_name(parent, asserted_result), asserted_result))
        assert(has_child_with_name(parent, asserted_result) == asserted_result)

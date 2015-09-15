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
import py_trees
import rocon_console.console as console

##############################################################################
# Logging Level
##############################################################################

py_trees.logging.level = py_trees.logging.Level.DEBUG
logger = py_trees.logging.get_logger("Nosetest")

##############################################################################
# Classes
##############################################################################

class Visitor:
    def __init__(self):
        self.logger = py_trees.logging.get_logger("Visitor")

    def initialise(self):
        pass

    def run(self, behaviour):
        self.logger.debug("  %s [visited][%s]" % (behaviour.name, behaviour.status))

def pre_tick_visitor(behaviour_tree):
    print("\n--------- Run %s ---------\n" % behaviour_tree.count)


def tick_tree(tree, visitor, from_iteration, to_iteration):
    print("\n================== Iteration %s-%s ==================\n" % (from_iteration, to_iteration))
    for i in range(from_iteration, to_iteration + 1):
        print("\n--------- Run %s ---------\n" % i)
        for node in tree.tick():
            node.visit(visitor)

def print_summary(nodes):
    print("\n--------- Summary ---------\n")
    for node in nodes:
        print("%s" % node)

@py_trees.meta.failure_is_success
class MustGoOnRegardless(py_trees.behaviours.Failure):
    pass

##############################################################################
# Tests
##############################################################################

def test_failure_is_success_tree():
    print(console.bold + "\n****************************************************************************************" + console.reset)
    print(console.bold + "* Failure is Success Tree" + console.reset)
    print(console.bold + "****************************************************************************************\n" + console.reset)
    root = py_trees.Selector("Root")
    failure = py_trees.behaviours.Failure("Failure")
    goon = MustGoOnRegardless("DontBeAfraidToBeTheGoon")
    root.add_child(failure)
    root.add_child(goon)
    py_trees.display.print_ascii_tree(root)
    visitor = Visitor()
    tick_tree(root, visitor, 1, 1)
      
    print("\n--------- Assertions ---------\n")
    print("root.status == py_trees.Status.SUCCESS")
    assert(root.status == py_trees.Status.SUCCESS)
    print("failure.status == py_trees.Status.FAILURE")
    assert(failure.status == py_trees.Status.FAILURE)
    print("goon.status == py_trees.Status.SUCCESS")
    assert(goon.status == py_trees.Status.SUCCESS)

      
def test_success_failure_tree():
    print(console.bold + "\n****************************************************************************************" + console.reset)
    print(console.bold + "* Success Failure Tree" + console.reset)
    print(console.bold + "****************************************************************************************\n" + console.reset)
    root = py_trees.Sequence("Root")
    selector = py_trees.Selector("Selector")
    failure = py_trees.behaviours.Failure("Failure")
    failure2 = py_trees.meta.inverter(py_trees.behaviours.Success("Failure2"))
    success = py_trees.behaviours.Success("Success")
    success2 = py_trees.meta.inverter(py_trees.behaviours.Failure("Success2"))
    selector.add_child(failure)
    selector.add_child(failure2)
    selector.add_child(success)
    root.add_child(selector)
    root.add_child(success2)
    py_trees.display.print_ascii_tree(root)
    visitor = Visitor()
    tick_tree(root, visitor, 1, 1)
      
    print("\n--------- Assertions ---------\n")
    print("success.status == py_trees.Status.SUCCESS")
    assert(success.status == py_trees.Status.SUCCESS)
    print("success2.status == py_trees.Status.SUCCESS")
    assert(success2.status == py_trees.Status.SUCCESS)
    print("root.status == py_trees.Status.SUCCESS")
    assert(root.status == py_trees.Status.SUCCESS)
    print("failure.status == py_trees.Status.FAILURE")
    assert(failure.status == py_trees.Status.FAILURE)
    print("failure2.status == py_trees.Status.FAILURE")
    assert(failure2.status == py_trees.Status.FAILURE)


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

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

import logging
from nose.tools import assert_raises
import py_trees
import rocon_console.console as console

##############################################################################
# Logging Level
##############################################################################

logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger("py_trees.Test")

##############################################################################
# Classes
##############################################################################

class Visitor:
    def __init__(self):
        self.logger = logging.getLogger("py_trees.Visitor")

    def run(self, behaviour):
        self.logger.debug("  %s [visited][%s]" % (behaviour.name, behaviour.status))

class PreTickVisitor:
    def run(self, behaviour_tree):
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

##############################################################################
# Tests
##############################################################################

def test_selector_composite():
    print(console.bold + "\n****************************************************************************************" + console.reset)
    print(console.bold + "* Selector" + console.reset)
    print(console.bold + "****************************************************************************************" + console.reset)
    visitor = Visitor()
    tree = py_trees.Selector(name='Selector')
    a = py_trees.Count(name="A")
    b = py_trees.Count(name="B")
    c = py_trees.Count(name="C", fail_until=0, running_until=3, success_until=15)
    tree.add_child(a)
    tree.add_child(b)
    tree.add_child(c)
    tick_tree(tree, visitor, 1, 3)
    print_summary(nodes=[a, b, c])
    print("--------- Assertions ---------\n")
    print("a.count == 3")
    assert(a.count == 3)
    print("a.status == py_trees.Status.FAILURE")
    assert(a.status == py_trees.Status.FAILURE)
    print("c.status == py_trees.Status.RUNNING")
    assert(c.status == py_trees.Status.RUNNING)
    print("c.number_count_resets == 0")
    assert(c.number_count_resets == 0)
    tick_tree(tree, visitor, 4, 4)
    print_summary(nodes=[a, b, c])
    print("--------- Assertions ---------\n")
    print("a.status == py_trees.Status.RUNNING")
    assert(a.status == py_trees.Status.RUNNING)
    print("c.number_count_resets == 1")
    assert(c.number_count_resets == 1)
    print("c.status == py_trees.Status.INVALID")
    assert(c.status == py_trees.Status.INVALID)
    tick_tree(tree, visitor, 5, 8)
    print_summary(nodes=[a, b, c])
    print("Done")

def test_sequence_composite():
    print(console.bold + "\n****************************************************************************************" + console.reset)
    print(console.bold + "* Sequence" + console.reset)
    print(console.bold + "****************************************************************************************" + console.reset)
    visitor = Visitor()
    tree = py_trees.Sequence(name='Sequence')
    a = py_trees.Count(name="A", fail_until=0, running_until=3, success_until=6)
    b = py_trees.Count(name="B", fail_until=0, running_until=3, success_until=6)
    c = py_trees.Count(name="C", fail_until=0, running_until=3, success_until=6)
    tree.add_child(a)
    tree.add_child(b)
    tree.add_child(c)
    tick_tree(tree, visitor, 1, 5)
    print_summary(nodes=[a, b, c])
    print("--------- Assertions ---------\n")
    print("a.status == py_trees.Status.SUCCESS")
    assert(a.status == py_trees.Status.SUCCESS)
    print("b.status == py_trees.Status.RUNNING")
    assert(b.status == py_trees.Status.RUNNING)
    print("tree.status == py_trees.Status.RUNNING")
    assert(tree.status == py_trees.Status.RUNNING)
    tick_tree(tree, visitor, 6, 10)
    print_summary(nodes=[a, b, c])
    print("--------- Assertions ---------\n")
    print("a.status == py_trees.Status.SUCCESS")
    assert(a.status == py_trees.Status.SUCCESS)
    print("b.status == py_trees.Status.SUCCESS")
    assert(b.status == py_trees.Status.SUCCESS)
    print("b.status == py_trees.Status.SUCCESS")
    assert(c.status == py_trees.Status.SUCCESS)
    print("tree.status == py_trees.Status.SUCCESS")
    assert(tree.status == py_trees.Status.SUCCESS)
    tick_tree(tree, visitor, 11, 13)
    print_summary(nodes=[a, b, c])
    print("--------- Assertions ---------\n")
    print("a.status == py_trees.Status.FAILURE")
    assert(a.status == py_trees.Status.FAILURE)
    print("tree.status == py_trees.Status.FAILURE")
    assert(tree.status == py_trees.Status.FAILURE)

def test_mixed_tree():
    print(console.bold + "\n****************************************************************************************" + console.reset)
    print(console.bold + "* Mixed Tree" + console.reset)
    print(console.bold + "****************************************************************************************" + console.reset)
    visitor = Visitor()
    
    a = py_trees.Count(name="A", fail_until=3, running_until=5, success_until=7)

    sequence = py_trees.Sequence(name="Sequence")
    b = py_trees.Count(name="B", fail_until=0, running_until=3, success_until=5)
    c = py_trees.Count(name="C", fail_until=0, running_until=3, success_until=5)
    sequence.add_child(b)
    sequence.add_child(c)

    d = py_trees.Count(name="D", fail_until=0, running_until=3, success_until=15)

    root = py_trees.Selector(name="Root")
    print("Root.children: %s " % [child.name for child in root.children])
    root.add_child(a)
    root.add_child(sequence)
    root.add_child(d)

    tick_tree(root, visitor, 1, 2)
    print_summary(nodes=[a, b, c, d])
    print("--------- Assertions ---------\n")
    print("a.status == py_trees.Status.FAILURE")
    assert(a.status == py_trees.Status.FAILURE)
    print("sequence.status == py_trees.Status.RUNNING")
    assert(sequence.status == py_trees.Status.RUNNING)
    print("b.status == py_trees.Status.RUNNING")
    assert(b.status == py_trees.Status.RUNNING)
    print("root.status == py_trees.Status.RUNNING")
    assert(root.status == py_trees.Status.RUNNING)

    tick_tree(root, visitor, 3, 11)
    print_summary(nodes=[a, b, c, d])
    print("--------- Assertions ---------\n")
    print("a.status == py_trees.Status.FAILURE")
    assert(a.status == py_trees.Status.FAILURE)
    print("sequence.status == py_trees.Status.SUCCESS")
    assert(sequence.status == py_trees.Status.SUCCESS)
    print("c.status == py_trees.Status.SUCCESS")
    assert(c.status == py_trees.Status.SUCCESS)
    print("root.status == py_trees.Status.SUCCESS")
    assert(root.status == py_trees.Status.SUCCESS)

    tick_tree(root, visitor, 12, 13)
    print_summary(nodes=[a, b, c, d])
    print("--------- Assertions ---------\n")
    print("a.status == py_trees.Status.FAILURE")
    assert(a.status == py_trees.Status.FAILURE)
    print("sequence.status == py_trees.Status.FAILURE")
    assert(sequence.status == py_trees.Status.FAILURE)
    print("b.status == py_trees.Status.FAILURE")
    assert(b.status == py_trees.Status.FAILURE)
    print("d.status == py_trees.Status.RUNNING")
    assert(d.status == py_trees.Status.RUNNING)
    print("root.status == py_trees.Status.RUNNING")
    assert(root.status == py_trees.Status.RUNNING)

def test_display():
    print(console.bold + "\n****************************************************************************************" + console.reset)
    print(console.bold + "* Display Tree" + console.reset)
    print(console.bold + "****************************************************************************************\n" + console.reset)

    a = py_trees.Count(name="A")
    sequence = py_trees.Sequence(name="Sequence")
    b = py_trees.Count(name="B")
    c = py_trees.Count(name="C")
    sequence.add_child(b)
    sequence.add_child(c)
    d = py_trees.Count(name="D")
    root = py_trees.Selector(name="Root")
    root.add_child(a)
    root.add_child(sequence)
    root.add_child(d)

    py_trees.display.print_ascii_tree(root)

    assert(True)

def test_full_iteration():
    print(console.bold + "\n****************************************************************************************" + console.reset)
    print(console.bold + "* Visit Whole Tree" + console.reset)
    print(console.bold + "****************************************************************************************\n" + console.reset)
    visitor = Visitor()

    a = py_trees.Count(name="A")
    sequence = py_trees.Sequence(name="Sequence")
    b = py_trees.Count(name="B")
    c = py_trees.Count(name="C")
    sequence.add_child(b)
    sequence.add_child(c)
    d = py_trees.Count(name="D")
    root = py_trees.Selector(name="Root")
    root.add_child(a)
    root.add_child(sequence)
    root.add_child(d)

    visitations = 0
    for child in root.iterate():
        visitations += 1
        child.visit(visitor)
    assert(visitations == 6)
    
def test_prune_behaviour_tree():
    print(console.bold + "\n****************************************************************************************" + console.reset)
    print(console.bold + "* Prune Behaviour Tree" + console.reset)
    print(console.bold + "****************************************************************************************\n" + console.reset)

    a = py_trees.Count(name="A")
    sequence = py_trees.Sequence(name="Sequence")
    b = py_trees.Count(name="B")
    c = py_trees.Count(name="C")
    sequence.add_child(b)
    sequence.add_child(c)
    d = py_trees.Count(name="D")
    root = py_trees.Selector(name="Root")
    root.add_child(a)
    root.add_child(sequence)
    root.add_child(d)
    
    tree = py_trees.BehaviourTree(root)
    py_trees.display.print_ascii_tree(tree.root)
    assert(len(sequence.children) == 2)
    tree.prune_subtree(c.id)
    py_trees.display.print_ascii_tree(tree.root)
    assert(len(sequence.children) == 1)
    tree.prune_subtree(sequence.id)
    py_trees.display.print_ascii_tree(tree.root)
    assert(len(root.children) == 2)
    
def test_replace_behaviour_tree():
    print(console.bold + "\n****************************************************************************************" + console.reset)
    print(console.bold + "* Replace Behaviour Subtree" + console.reset)
    print(console.bold + "****************************************************************************************\n" + console.reset)

    a = py_trees.Count(name="A")
    sequence1 = py_trees.Sequence(name="Sequence1")
    b = py_trees.Count(name="B")
    c = py_trees.Count(name="C")
    sequence1.add_child(b)
    sequence1.add_child(c)
    d = py_trees.Count(name="D")
    root = py_trees.Selector(name="Root")
    root.add_child(a)
    root.add_child(sequence1)
    root.add_child(d)
    
    tree = py_trees.BehaviourTree(root)
    py_trees.display.print_ascii_tree(tree.root)
    assert(len(sequence1.children) == 2)

    sequence2 = py_trees.Sequence(name="Sequence2")
    e = py_trees.Count(name="E")
    f = py_trees.Count(name="F")
    g = py_trees.Count(name="G")
    sequence2.add_child(e)
    sequence2.add_child(f)
    sequence2.add_child(g)

    tree.replace_subtree(sequence1.id, sequence2)
    py_trees.display.print_ascii_tree(tree.root)
    assert(len(sequence2.children) == 3)

def test_tick_tock_behaviour_tree():
    print(console.bold + "\n****************************************************************************************" + console.reset)
    print(console.bold + "* Tick Tock Behaviour Tree" + console.reset)
    print(console.bold + "****************************************************************************************\n" + console.reset)

    a = py_trees.Count(name="A")
    sequence = py_trees.Sequence(name="Sequence")
    b = py_trees.Count(name="B")
    c = py_trees.Count(name="C")
    sequence.add_child(b)
    sequence.add_child(c)
    d = py_trees.Count(name="D")
    root = py_trees.Selector(name="Root")
    root.add_child(a)
    root.add_child(sequence)
    root.add_child(d)
    
    tree = py_trees.BehaviourTree(root)
    py_trees.display.print_ascii_tree(tree.root)

    tree.visitors.append(Visitor())
    tree.tick_tock(100, 5, pre_tick_visitor=PreTickVisitor())

    print("\n--------- Assertions ---------\n")
    print("a.status == py_trees.Status.RUNNING")
    assert(a.status == py_trees.Status.RUNNING)
    
def test_success_failure_tree():
    print(console.bold + "\n****************************************************************************************" + console.reset)
    print(console.bold + "* Success Failure Tree" + console.reset)
    print(console.bold + "****************************************************************************************\n" + console.reset)
    root = py_trees.Selector("Root")
    failure = py_trees.Failure("Failure")
    success = py_trees.Success("Success")
    root.add_child(failure)
    root.add_child(success)
    py_trees.display.print_ascii_tree(root)
    visitor = Visitor()
    tick_tree(root, visitor, 1, 1)
    
    print("\n--------- Assertions ---------\n")
    print("success.status == py_trees.Status.SUCCESS")
    assert(success.status == py_trees.Status.SUCCESS)
    print("root.status == py_trees.Status.SUCCESS")
    assert(root.status == py_trees.Status.SUCCESS)
    print("failure.status == py_trees.Status.FAILURE")
    assert(failure.status == py_trees.Status.FAILURE)


# def test_foo():
#     print('--------- Nosetest Logs ---------')
#     print_logging()
#     py_trees.foo1()
#     py_trees.foo2()
#     print('--------- Behaviour Logs ---------')
#     d = py_trees.Count(name="D")
#     d.initialise()
#     print("Done")
#     assert(True)

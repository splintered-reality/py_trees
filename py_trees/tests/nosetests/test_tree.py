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

class Count(py_trees.Behaviour):
    def __init__(self, name="Count", fail_until=3, running_until=5, success_until=6, *args, **kwargs):
        super(Count, self).__init__(name, *args, **kwargs)
        self.count = 0
        self.fail_until = fail_until
        self.running_until = running_until
        self.success_until = success_until
        self.number_count_resets = 0
        self.number_updated = 0
        self.logger = logging.getLogger("py_trees.Count")

    def terminate(self, new_status):
        self.logger.debug("  %s [terminate()]" % self.name)
        # reset only if udpate got us into an invalid state
        if new_status == py_trees.Status.INVALID:
            self.count = 0
            self.number_count_resets += 1

    def update(self):
        self.number_updated += 1
        self.count += 1
        if self.count <= self.fail_until:
            self.logger.debug("  %s [update()][%s -> FAILURE]" % (self.name, self.count))
            return py_trees.Status.FAILURE
        elif self.count <= self.running_until:
            self.logger.debug("  %s [update()][%s -> RUNNING]" % (self.name, self.count))
            return py_trees.Status.RUNNING
        elif self.count <= self.success_until:
            self.logger.debug("  %s [update()][%s -> SUCCESS]" % (self.name, self.count))
            return py_trees.Status.SUCCESS
        else:
            self.logger.debug("  %s [update()][%s -> FAILURE]" % (self.name, self.count))
            return py_trees.Status.FAILURE
    
    def __repr__(self):
        s =  "%s\n" % self.name
        s += "  Status : %s\n" % self.status
        s += "  Count  : %s\n" % self.count
        s += "  Resets : %s\n" % self.number_count_resets
        s += "  Updates: %s\n" % self.number_updated
        return s

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
    a = Count(name="A")
    b = Count(name="B")
    c = Count(name="C", fail_until=0, running_until=3, success_until=15)
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
    a = Count(name="A", fail_until=0, running_until=3, success_until=6)
    b = Count(name="B", fail_until=0, running_until=3, success_until=6)
    c = Count(name="C", fail_until=0, running_until=3, success_until=6)
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
    
    a = Count(name="A", fail_until=3, running_until=5, success_until=7)

    sequence = py_trees.Sequence(name="Sequence")
    b = Count(name="B", fail_until=0, running_until=3, success_until=5)
    c = Count(name="C", fail_until=0, running_until=3, success_until=5)
    sequence.add_child(b)
    sequence.add_child(c)

    d = Count(name="D", fail_until=0, running_until=3, success_until=15)

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

    a = Count(name="A")
    sequence = py_trees.Sequence(name="Sequence")
    b = Count(name="B")
    c = Count(name="C")
    sequence.add_child(b)
    sequence.add_child(c)
    d = Count(name="D")
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

    a = Count(name="A")
    sequence = py_trees.Sequence(name="Sequence")
    b = Count(name="B")
    c = Count(name="C")
    sequence.add_child(b)
    sequence.add_child(c)
    d = Count(name="D")
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

    a = Count(name="A")
    sequence = py_trees.Sequence(name="Sequence")
    b = Count(name="B")
    c = Count(name="C")
    sequence.add_child(b)
    sequence.add_child(c)
    d = Count(name="D")
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

    a = Count(name="A")
    sequence1 = py_trees.Sequence(name="Sequence1")
    b = Count(name="B")
    c = Count(name="C")
    sequence1.add_child(b)
    sequence1.add_child(c)
    d = Count(name="D")
    root = py_trees.Selector(name="Root")
    root.add_child(a)
    root.add_child(sequence1)
    root.add_child(d)
    
    tree = py_trees.BehaviourTree(root)
    py_trees.display.print_ascii_tree(tree.root)
    assert(len(sequence1.children) == 2)

    sequence2 = py_trees.Sequence(name="Sequence2")
    e = Count(name="E")
    f = Count(name="F")
    g = Count(name="G")
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

    a = Count(name="A")
    sequence = py_trees.Sequence(name="Sequence")
    b = Count(name="B")
    c = Count(name="C")
    sequence.add_child(b)
    sequence.add_child(c)
    d = Count(name="D")
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
    

# def test_foo():
#     print('--------- Nosetest Logs ---------')
#     print_logging()
#     py_trees.foo1()
#     py_trees.foo2()
#     print('--------- Behaviour Logs ---------')
#     d = Count(name="D")
#     d.initialise()
#     print("Done")
#     assert(True)

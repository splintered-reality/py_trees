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

##############################################################################
# Tests
##############################################################################

def test_selector_composite():
    print(console.bold + "\n****************************************************************************************" + console.reset)
    print(console.bold + "* Selector" + console.reset)
    print(console.bold + "****************************************************************************************" + console.reset)
    visitor = py_trees.trees.DebugVisitor()
    tree = py_trees.Selector(name='Selector')
    a = py_trees.behaviours.Count(name="A")
    b = py_trees.behaviours.Count(name="B")
    c = py_trees.behaviours.Count(name="C", fail_until=0, running_until=3, success_until=15)
    tree.add_child(a)
    tree.add_child(b)
    tree.add_child(c)
    py_trees.display.print_ascii_tree(tree, 0)
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
    visitor = py_trees.trees.DebugVisitor()
    tree = py_trees.Sequence(name='Sequence')
    a = py_trees.behaviours.Count(name="A", fail_until=0, running_until=3, success_until=6)
    b = py_trees.behaviours.Count(name="B", fail_until=0, running_until=3, success_until=6)
    c = py_trees.behaviours.Count(name="C", fail_until=0, running_until=3, success_until=6)
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
    visitor = py_trees.trees.DebugVisitor()

    a = py_trees.behaviours.Count(name="A", fail_until=3, running_until=5, success_until=7)

    sequence = py_trees.Sequence(name="Sequence")
    b = py_trees.behaviours.Count(name="B", fail_until=0, running_until=3, success_until=5)
    c = py_trees.behaviours.Count(name="C", fail_until=0, running_until=3, success_until=5)
    sequence.add_child(b)
    sequence.add_child(c)

    d = py_trees.behaviours.Count(name="D", fail_until=0, running_until=3, success_until=15)

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
    print("sequence.status == py_trees.Status.RUNNING")
    assert(sequence.status == py_trees.Status.RUNNING)
    print("c.status == py_trees.Status.RUNNING")
    assert(c.status == py_trees.Status.RUNNING)
    print("root.status == py_trees.Status.RUNNING")
    assert(root.status == py_trees.Status.RUNNING)

    tick_tree(root, visitor, 12, 14)
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

    tick_tree(root, visitor, 15, 16)
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

    a = py_trees.behaviours.Count(name="A")
    sequence = py_trees.Sequence(name="Sequence")
    b = py_trees.behaviours.Count(name="B")
    c = py_trees.behaviours.Count(name="C")
    sequence.add_child(b)
    sequence.add_child(c)
    d = py_trees.behaviours.Count(name="D")
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

    visitor = py_trees.trees.DebugVisitor()
    a = py_trees.behaviours.Count(name="A")
    sequence = py_trees.Sequence(name="Sequence")
    b = py_trees.behaviours.Count(name="B")
    c = py_trees.behaviours.Count(name="C")
    sequence.add_child(b)
    sequence.add_child(c)
    d = py_trees.behaviours.Count(name="D")
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

    a = py_trees.behaviours.Count(name="A")
    sequence = py_trees.Sequence(name="Sequence")
    b = py_trees.behaviours.Count(name="B")
    c = py_trees.behaviours.Count(name="C")
    sequence.add_child(b)
    sequence.add_child(c)
    d = py_trees.behaviours.Count(name="D")
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

    a = py_trees.behaviours.Count(name="A")
    sequence1 = py_trees.Sequence(name="Sequence1")
    b = py_trees.behaviours.Count(name="B")
    c = py_trees.behaviours.Count(name="C")
    sequence1.add_child(b)
    sequence1.add_child(c)
    d = py_trees.behaviours.Count(name="D")
    root = py_trees.Selector(name="Root")
    root.add_child(a)
    root.add_child(sequence1)
    root.add_child(d)

    tree = py_trees.BehaviourTree(root)
    py_trees.display.print_ascii_tree(tree.root)
    assert(len(sequence1.children) == 2)

    sequence2 = py_trees.Sequence(name="Sequence2")
    e = py_trees.behaviours.Count(name="E")
    f = py_trees.behaviours.Count(name="F")
    g = py_trees.behaviours.Count(name="G")
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

    a = py_trees.behaviours.Count(name="A")
    sequence = py_trees.Sequence(name="Sequence")
    b = py_trees.behaviours.Count(name="B")
    c = py_trees.behaviours.Count(name="C")
    sequence.add_child(b)
    sequence.add_child(c)
    d = py_trees.behaviours.Count(name="D")
    root = py_trees.Selector(name="Root")
    root.add_child(a)
    root.add_child(sequence)
    root.add_child(d)

    tree = py_trees.BehaviourTree(root)
    py_trees.display.print_ascii_tree(tree.root)

    visitor = py_trees.trees.DebugVisitor()
    tree.visitors.append(visitor)
    tree.tick_tock(100, 5, pre_tick_handler=pre_tick_visitor)

    print("\n--------- Assertions ---------\n")
    print("a.status == py_trees.Status.RUNNING")
    assert(a.status == py_trees.Status.RUNNING)

def test_success_failure_tree():
    print(console.bold + "\n****************************************************************************************" + console.reset)
    print(console.bold + "* Success Failure Tree" + console.reset)
    print(console.bold + "****************************************************************************************\n" + console.reset)
    root = py_trees.Selector("Root")
    failure = py_trees.behaviours.Failure("Failure")
    failure2 = py_trees.meta.inverter(py_trees.behaviours.Success("Failure2"))
    success = py_trees.behaviours.Success("Success")
    root.add_child(failure)
    root.add_child(failure2)
    root.add_child(success)
    py_trees.display.print_ascii_tree(root)
    visitor = py_trees.trees.DebugVisitor()
    tick_tree(root, visitor, 1, 1)

    print("\n--------- Assertions ---------\n")
    print("success.status == py_trees.Status.SUCCESS")
    assert(success.status == py_trees.Status.SUCCESS)
    print("root.status == py_trees.Status.SUCCESS")
    assert(root.status == py_trees.Status.SUCCESS)
    print("failure.status == py_trees.Status.FAILURE")
    assert(failure.status == py_trees.Status.FAILURE)
    print("failure2.status == py_trees.Status.FAILURE")
    assert(failure2.status == py_trees.Status.FAILURE)

def test_tip_simple():
    print(console.bold + "\n****************************************************************************************" + console.reset)
    print(console.bold + "* Tip Simple" + console.reset)
    print(console.bold + "****************************************************************************************\n" + console.reset)

    # behaviours will be running the first time they are seen, then success for subsequent ticks
    seq = py_trees.composites.Sequence(name="Sequence")
    a = py_trees.behaviours.Count(name="A", fail_until=0, running_until=1, success_until=100)
    b = py_trees.behaviours.Count(name="B", fail_until=0, running_until=1, success_until=100)
    seq.add_child(a)
    seq.add_child(b)

    tree = py_trees.BehaviourTree(seq)
    py_trees.display.print_ascii_tree(tree.root)

    visitor = py_trees.trees.DebugVisitor()
    tree.visitors.append(visitor)

    print("\n--------- Assertions (before initialisation) ---------\n")
    # an uninitialised tree/behaviour always has a tip of None
    assert(tree.root.tip() == None)
    assert(seq.tip() == None)
    assert(a.tip() == None)
    assert(b.tip() == None)

    tree.tick()
    print("\n--------- Assertions ---------\n")
    assert(a.status == py_trees.Status.RUNNING)
    assert(b.status == py_trees.Status.INVALID)
    # the root of sequence and tree should be the currently running node
    assert(tree.root.tip() == a)
    assert(seq.tip() == a)
    # when a node is running/has run, its tip is itself
    assert(a.tip() == a)
    assert(b.tip() == None)

    tree.tick()
    print("\n--------- Assertions ---------\n")
    assert(a.status == py_trees.Status.SUCCESS)
    assert(b.status == py_trees.Status.RUNNING)
    # the root of sequence and tree should be the currently running node
    assert(tree.root.tip() == b)
    assert(seq.tip() == b)
    # when a node is running/has run, its tip is itself
    assert(a.tip() == a)
    assert(b.tip() == b)

    tree.tick()
    print("\n--------- Assertions ---------\n")
    assert(a.status == py_trees.Status.SUCCESS)
    assert(b.status == py_trees.Status.SUCCESS)
    # the root of sequence and tree should be the currently running node
    assert(tree.root.tip() == b)
    assert(seq.tip() == b)
    # when a node is running/has run, its tip is itself
    assert(a.tip() == a)
    assert(b.tip() == b)

def test_tip_complex():
    print(console.bold + "\n****************************************************************************************" + console.reset)
    print(console.bold + "* Tip Complex" + console.reset)
    print(console.bold + "****************************************************************************************\n" + console.reset)

    # behaviours will be running the first time they are seen, then success for subsequent ticks
    sel = py_trees.composites.Selector(name="Selector")
    seq1 = py_trees.composites.Sequence(name="Sequence1")
    seq2 = py_trees.composites.Sequence(name="Sequence2")

    # selector left branch fails the two times, so seq2 behaviours run. The
    # third time it is running, stopping seq2
    a = py_trees.behaviours.Count(name="A", fail_until=1, running_until=0, success_until=10)
    b = py_trees.behaviours.Count(name="B", fail_until=0, running_until=1, success_until=10)
    c = py_trees.behaviours.Count(name="C", fail_until=0, running_until=2, success_until=10)
    d = py_trees.behaviours.Count(name="D", fail_until=0, running_until=1, success_until=10)

    seq1.add_child(a)
    seq1.add_child(b)
    seq2.add_child(c)
    seq2.add_child(d)

    sel.add_child(seq1)
    sel.add_child(seq2)

    tree = py_trees.BehaviourTree(sel)
    py_trees.display.print_ascii_tree(tree.root)

    visitor = py_trees.trees.DebugVisitor()
    tree.visitors.append(visitor)
    tree.tick()

    print("\n--------- Assertions ---------\n")
    assert(a.status == py_trees.Status.FAILURE)
    assert(b.status == py_trees.Status.INVALID)
    assert(c.status == py_trees.Status.RUNNING)
    assert(d.status == py_trees.Status.INVALID)

    # the root of sequence and tree should be the currently running node
    assert(seq1.tip() == a)
    assert(seq2.tip() == c)
    assert(tree.root.tip() == c)

    tree.tick()
    print("\n--------- Assertions ---------\n")
    assert(a.status == py_trees.Status.SUCCESS)
    assert(b.status == py_trees.Status.RUNNING)
    assert(c.status == py_trees.Status.INVALID)
    assert(d.status == py_trees.Status.INVALID)


    assert(seq1.tip() == b)
    assert(seq2.tip() == None)
    assert(tree.root.tip() == b)

    tree.tick()

def test_condition():
    print(console.bold + "\n****************************************************************************************" + console.reset)
    print(console.bold + "* Condition" + console.reset)
    print(console.bold + "****************************************************************************************\n" + console.reset)

    # behaviours will be running the first time they are seen, then success for subsequent ticks

    d = py_trees.behaviours.Count(name="D", fail_until=2, running_until=2, success_until=10)
    condition = py_trees.behaviours.Condition('condition', d, py_trees.Status.SUCCESS)

    tree = py_trees.BehaviourTree(condition)
    py_trees.display.print_ascii_tree(tree.root)

    visitor = py_trees.trees.DebugVisitor()
    tree.visitors.append(visitor)
    tree.tick()

    print("\n--------- Assertions ---------\n")
    assert(condition.status == py_trees.Status.RUNNING)
    assert(d.status == py_trees.Status.FAILURE)

    tree.tick()

    print("\n--------- Assertions ---------\n")
    assert(condition.status == py_trees.Status.RUNNING)
    assert(d.status == py_trees.Status.FAILURE)

    tree.tick()

    print("\n--------- Assertions ---------\n")
    assert(condition.status == py_trees.Status.SUCCESS)
    assert(d.status == py_trees.Status.SUCCESS)

def test_failed_tree():
    print(console.bold + "\n****************************************************************************************" + console.reset)
    print(console.bold + "* Failed Tree" + console.reset)
    print(console.bold + "****************************************************************************************\n" + console.reset)

    root = py_trees.Selector("Root")
    f1 = py_trees.behaviours.Failure("Failure 1")
    f2 = py_trees.behaviours.Failure("Failure 2")
    f3 = py_trees.behaviours.Failure("Failure 3")
    root.add_child(f1)
    root.add_child(f2)
    root.add_child(f3)
    tree = py_trees.BehaviourTree(root)
    tree.tick()
    print("\n--------- Assertions ---------\n")
    print("root.tip().name == Failure 3")
    assert(root.tip().name == "Failure 3")

    # TODO failed sequence tree

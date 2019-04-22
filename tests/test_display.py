#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/splintered-reality/py_trees/devel/LICENSE
#

##############################################################################
# Imports
##############################################################################

import functools
import py_trees
import py_trees.console as console
import py_trees.display as display
import xml.etree.ElementTree

##############################################################################
# Tests
##############################################################################


def test_symbols():
    console.banner("Symbols")
    # This test has no assertions, except from the human eye. When it fails,
    # ticks and crosses fail and must be inspected by the human eye
    if console.has_unicode():
        symbol_groups = (display.html_symbols, display.unicode_symbols, display.ascii_symbols)
    else:
        symbol_groups = (display.html_symbols, display.ascii_symbols)

    for symbols in symbol_groups:

        print("Status: [{0}][{1}][{2}][{3}]".format(
            symbols[py_trees.common.Status.SUCCESS],
            symbols[py_trees.common.Status.FAILURE],
            symbols[py_trees.common.Status.INVALID],
            symbols[py_trees.common.Status.RUNNING]
            )
        )

        print("Classes: [{0}][{1}][{2}][{3}]".format(
            symbols[py_trees.behaviours.Behaviour],
            symbols[py_trees.composites.Sequence],
            symbols[py_trees.composites.Selector],
            symbols[py_trees.composites.Parallel]
            )
        )


def test_html_tree():
    console.banner("Ascii/Html Snapshots - Comparison Check")

    def print_tree(snapshot_visitor, tree):
        print(
            py_trees.display.ascii_tree(
                tree.root,
                visited=snapshot_visitor.visited,
                previously_visited=snapshot_visitor.previously_visited
            )
        )
        html_snippet = py_trees.display.html_tree(
            tree.root,
            visited=snapshot_visitor.visited,
            previously_visited=snapshot_visitor.previously_visited
        )
        print(html_snippet)
        print()
        try:
            unused_element = xml.etree.ElementTree.fromstring(html_snippet)
        except xml.etree.ElementTree.ParseError:
            assert False, "failed to parse the xhtml snippet as valid xml"

    root = py_trees.composites.Selector("Selector")
    root.add_child(
        py_trees.behaviours.Count(
            name="High Priority",
            fail_until=1,
            running_until=1,
            success_until=10
        )
    )
    root.add_child(py_trees.behaviours.Running(name="Low Priority"))
    tree = py_trees.trees.BehaviourTree(root)
    snapshot_visitor = py_trees.visitors.SnapshotVisitor()
    tree.visitors.append(snapshot_visitor)
    tree.add_post_tick_handler(functools.partial(print_tree, snapshot_visitor))
    tree.tick()
    tree.tick()


def test_ascii_snapshot_priority_interrupt():
    """
    Tickle the ascii tree generator, but also check specifically
    that a running node that is priority interrupted still gets
    shown as invalidated ('-') via the visitor mechanisms.
    """
    console.banner("Ascii Snapshots - Priority Interrupt")

    def post_tick_handler(snapshot_visitor, tree):
        print(
            py_trees.display.ascii_tree(
                tree.root,
                visited=snapshot_visitor.visited,
                previously_visited=snapshot_visitor.previously_visited
            )
        )

    root = py_trees.composites.Selector("Selector")
    root.add_child(
        py_trees.behaviours.Count(
            name="High Priority",
            fail_until=1,
            running_until=1,
            success_until=10
        )
    )
    root.add_child(py_trees.behaviours.Running(name="Low Priority"))
    tree = py_trees.trees.BehaviourTree(root)
    snapshot_visitor = py_trees.visitors.SnapshotVisitor()
    tree.visitors.append(snapshot_visitor)
    tree.add_post_tick_handler(functools.partial(post_tick_handler, snapshot_visitor))
    tree.tick()
    tree.tick()
    last_line = py_trees.display.ascii_tree(
        tree.root,
        visited=snapshot_visitor.visited,
        previously_visited=snapshot_visitor.previously_visited
    ).splitlines()[-1]
    print("\n--------- Assertions ---------\n")
    print("Invalidated Lower Priority Symbol '-' is displayed")
    assert('-' in last_line)

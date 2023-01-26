#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/splintered-reality/py_trees/devel/LICENSE
#

##############################################################################
# Imports
##############################################################################

import functools
import xml.etree.ElementTree

import py_trees
import py_trees.console as console
import py_trees.display as display

##############################################################################
# Tests
##############################################################################


def test_symbols() -> None:
    console.banner("Symbols")
    # This test has no assertions, except from the human eye. When it fails,
    # ticks and crosses fail and must be inspected by the human eye
    if console.has_unicode():
        symbol_groups = [
            display.xhtml_symbols,
            display.unicode_symbols,
            display.ascii_symbols,
        ]
    else:
        symbol_groups = [display.xhtml_symbols, display.ascii_symbols]

    for symbols in symbol_groups:
        print(
            "Status: [{0}][{1}][{2}][{3}]".format(
                symbols[py_trees.common.Status.SUCCESS],
                symbols[py_trees.common.Status.FAILURE],
                symbols[py_trees.common.Status.INVALID],
                symbols[py_trees.common.Status.RUNNING],
            )
        )

        print(
            "Classes: [{0}][{1}][{2}][{3}]".format(
                symbols["behaviour"],
                symbols["sequence_with_memory"],
                symbols["selector_with_memory"],
                symbols["parallel"],
            )
        )


def test_text_trees() -> None:
    console.banner("Text Trees")

    root = py_trees.composites.Selector(name="Selector", memory=False)
    high_priority = py_trees.behaviours.StatusQueue(
        name="High Priority",
        queue=[py_trees.common.Status.FAILURE],
        eventually=py_trees.common.Status.SUCCESS,
    )
    low_priority = py_trees.behaviours.Running(name="Low Priority")
    selector_with_memory = py_trees.composites.Selector(
        name="Selector w/ Memory", memory=True
    )
    sequence_with_memory = py_trees.composites.Sequence(
        name="Sequence w/ Memory", memory=True
    )
    success = py_trees.behaviours.Success("Success")
    selector_with_memory.add_child(success)
    root.add_children(
        [high_priority, low_priority, selector_with_memory, sequence_with_memory]
    )
    tree = py_trees.trees.BehaviourTree(root)
    snapshot_visitor = py_trees.visitors.SnapshotVisitor()
    tree.visitors.append(snapshot_visitor)
    tree.tick()
    tree.tick()
    snippets = {}
    # Visited
    snippets["visited_ascii_tree"] = display.ascii_tree(
        tree.root,
        visited=snapshot_visitor.visited,
        previously_visited=snapshot_visitor.previously_visited,
    )
    print(snippets["visited_ascii_tree"])
    snippets["visited_xhtml"] = display.xhtml_tree(
        tree.root,
        visited=snapshot_visitor.visited,
        previously_visited=snapshot_visitor.previously_visited,
    )
    print(snippets["visited_xhtml"])
    print()
    # Non-Visited
    snippets["non_visited_ascii_tree"] = display.ascii_tree(tree.root)
    print(snippets["non_visited_ascii_tree"])
    print()
    snippets["non_visited_xhtml"] = display.xhtml_tree(tree.root)
    print(snippets["non_visited_xhtml"])
    print()

    # Non-Visited with Status
    snippets["non_visited_ascii_tree_status"] = display.ascii_tree(
        tree.root, show_status=True
    )
    print(snippets["non_visited_ascii_tree_status"])
    print()
    snippets["non_visited_xhtml_status"] = display.xhtml_tree(
        tree.root, show_status=True
    )
    print(snippets["non_visited_xhtml_status"])
    print()

    py_trees.tests.print_assert_banner()
    for snippet_name, snippet in snippets.items():
        for b in root.iterate():
            py_trees.tests.print_assert_details(
                "{} in {}".format(b.name, snippet_name), True, b.name in snippet
            )
            assert b.name in snippet

    py_trees.tests.print_assert_details(
        "status symbol '-' in visited_ascii_tree",
        True,
        "-" in snippets["visited_ascii_tree"],
    )
    assert "-" in snippets["visited_ascii_tree"]
    py_trees.tests.print_assert_details(
        "status symbol '-' in non_visited_ascii_tree_status",
        True,
        "-" in snippets["non_visited_ascii_tree_status"],
    )
    assert "-" in snippets["non_visited_ascii_tree_status"]
    try:
        unused_element = xml.etree.ElementTree.fromstring(  # noqa: F841 [unused]
            snippets["visited_xhtml"]
        )
        unused_element = xml.etree.ElementTree.fromstring(  # noqa: F841 [unused]
            snippets["non_visited_xhtml"]
        )
        unused_element = xml.etree.ElementTree.fromstring(  # noqa: F841 [unused]
            snippets["non_visited_xhtml_status"]
        )
        py_trees.tests.print_assert_details("xml ParseError", None, None)
    except xml.etree.ElementTree.ParseError as e:
        py_trees.tests.print_assert_details("xml ParseError", None, str(e))
        assert False, "failed to parse the xhtml snippet as valid xml"


def test_ascii_snapshot_priority_interrupt() -> None:
    """
    Tickle the ascii tree generator, but also check specifically
    that a running node that is priority interrupted still gets
    shown as invalidated ('-') via the visitor mechanisms.
    """
    console.banner("Ascii Snapshots - Priority Interrupt")

    if console.has_unicode():
        text_tree = (
            display.unicode_tree if console.has_unicode() else display.ascii_tree
        )

    def post_tick_handler(
        snapshot_visitor: py_trees.visitors.SnapshotVisitor,
        tree: py_trees.trees.BehaviourTree,
    ) -> None:
        print(
            text_tree(
                tree.root,
                visited=snapshot_visitor.visited,
                previously_visited=snapshot_visitor.previously_visited,
            )
        )

    root = py_trees.composites.Selector(name="Selector", memory=False)
    root.add_child(
        py_trees.behaviours.StatusQueue(
            name="High Priority",
            queue=[py_trees.common.Status.FAILURE],
            eventually=py_trees.common.Status.SUCCESS,
        )
    )
    root.add_child(py_trees.behaviours.Running(name="Low Priority"))
    tree = py_trees.trees.BehaviourTree(root)
    snapshot_visitor = py_trees.visitors.SnapshotVisitor()
    tree.visitors.append(snapshot_visitor)
    tree.add_post_tick_handler(functools.partial(post_tick_handler, snapshot_visitor))
    tree.tick()
    tree.tick()
    last_line = text_tree(
        tree.root,
        visited=snapshot_visitor.visited,
        previously_visited=snapshot_visitor.previously_visited,
    ).splitlines()[-1]
    print("\n--------- Assertions ---------\n")
    print("Invalidated Lower Priority Symbol '-' is displayed")
    assert "-" in last_line

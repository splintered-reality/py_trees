#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/splintered-reality/py_trees/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
Demonstrates a simple logging capability.

.. argparse::
   :module: py_trees.demos.logging
   :func: command_line_argument_parser
   :prog: py-trees-demo-logging

.. graphviz:: dot/demo-logging.dot

.. image:: images/logging.gif
"""

##############################################################################
# Imports
##############################################################################

import argparse
import functools
import json
import sys
import time
import typing

import py_trees
import py_trees.console as console

##############################################################################
# Classes
##############################################################################


def description(root: py_trees.behaviour.Behaviour) -> str:
    """
    Print description and usage information about the program.

    Returns:
       the program description string
    """
    content = "A demonstration of logging with trees.\n\n"
    content += "This demo utilises a SnapshotVisitor to trigger\n"
    content += "a post-tick handler to dump a serialisation of the\n"
    content += "tree to a json log file.\n"
    content += "\n"
    content += "This coupling of visitor and post-tick handler can be\n"
    content += "used for any kind of event handling - the visitor is the\n"
    content += "trigger and the post-tick handler the action. Aside from\n"
    content += "logging, the most common use case is to serialise the tree\n"
    content += "for messaging to a graphical, runtime monitor.\n"
    content += "\n"
    if py_trees.console.has_colours:
        banner_line = console.green + "*" * 79 + "\n" + console.reset
        s = banner_line
        s += console.bold_white + "Logging".center(79) + "\n" + console.reset
        s += banner_line
        s += "\n"
        s += content
        s += "\n"
        s += py_trees.display.unicode_tree(root)
        s += "\n"
        s += banner_line
    else:
        s = content
    return s


def epilog() -> typing.Optional[str]:
    """
    Print a noodly epilog for --help.

    Returns:
       the noodly message
    """
    if py_trees.console.has_colours:
        return (
            console.cyan
            + "And his noodly appendage reached forth to tickle the blessed...\n"
            + console.reset
        )
    else:
        return None


def command_line_argument_parser() -> argparse.ArgumentParser:
    """
    Process command line arguments.

    Returns:
        the argument parser
    """
    parser = argparse.ArgumentParser(
        description=description(create_tree()),
        epilog=epilog(),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    group = parser.add_mutually_exclusive_group()
    group.add_argument(
        "-r", "--render", action="store_true", help="render dot tree to file"
    )
    group.add_argument(
        "-i",
        "--interactive",
        action="store_true",
        help="pause and wait for keypress at each tick",
    )
    return parser


def logger(
    snapshot_visitor: py_trees.visitors.DisplaySnapshotVisitor,
    behaviour_tree: py_trees.trees.BehaviourTree,
) -> None:
    """Log the tree (relevant parts thereof) to a yaml file.

    Use as a post-tick handler for a tree.
    """
    if snapshot_visitor.changed:
        print(console.cyan + "Logging.......................yes\n" + console.reset)
        tree_serialisation = {"tick": behaviour_tree.count, "nodes": []}
        for node in behaviour_tree.root.iterate():
            node_type_str = "Behaviour"
            for behaviour_type in [
                py_trees.composites.Sequence,
                py_trees.composites.Selector,
                py_trees.composites.Parallel,
                py_trees.decorators.Decorator,
            ]:
                if isinstance(node, behaviour_type):
                    node_type_str = behaviour_type.__name__
            if node.tip() is not None:
                node_tip = node.tip()
                assert node_tip is not None  # help mypy
                node_tip_id = str(node_tip.id)
            else:
                node_tip_id = "none"
            node_snapshot = {
                "name": node.name,
                "id": str(node.id),
                "parent_id": str(node.parent.id) if node.parent else "none",
                "child_ids": [str(child.id) for child in node.children],
                "tip_id": node_tip_id,
                "class_name": str(node.__module__) + "." + str(type(node).__name__),
                "type": node_type_str,
                "status": node.status.value,
                "message": node.feedback_message,
                "is_active": True if node.id in snapshot_visitor.visited else False,
            }
            typing.cast(list, tree_serialisation["nodes"]).append(node_snapshot)
        if behaviour_tree.count == 0:
            with open("dump.json", "w+") as outfile:
                json.dump(tree_serialisation, outfile, indent=4)
        else:
            with open("dump.json", "a") as outfile:
                json.dump(tree_serialisation, outfile, indent=4)
    else:
        print(console.yellow + "Logging.......................no\n" + console.reset)


def create_tree() -> py_trees.behaviour.Behaviour:
    """
    Create the root behaviour and it's subtree.

    Returns:
        the root behaviour
    """
    every_n_success = py_trees.behaviours.SuccessEveryN("EveryN", 5)
    sequence = py_trees.composites.Sequence(name="Sequence", memory=True)
    guard = py_trees.behaviours.Success("Guard")
    periodic_success = py_trees.behaviours.Periodic("Periodic", 3)
    finisher = py_trees.behaviours.Success("Finisher")
    sequence.add_child(guard)
    sequence.add_child(periodic_success)
    sequence.add_child(finisher)
    sequence.blackbox_level = py_trees.common.BlackBoxLevel.COMPONENT
    idle = py_trees.behaviours.Success("Idle")
    root = py_trees.composites.Selector(name="Logging", memory=False)
    root.add_child(every_n_success)
    root.add_child(sequence)
    root.add_child(idle)
    return root


##############################################################################
# Main
##############################################################################


def main() -> None:
    """Entry point for the demo script."""
    args = command_line_argument_parser().parse_args()
    py_trees.logging.level = py_trees.logging.Level.DEBUG
    tree = create_tree()
    print(description(tree))

    ####################
    # Rendering
    ####################
    if args.render:
        py_trees.display.render_dot_tree(tree)
        sys.exit()

    ####################
    # Tree Stewardship
    ####################
    behaviour_tree = py_trees.trees.BehaviourTree(tree)

    debug_visitor = py_trees.visitors.DebugVisitor()
    snapshot_visitor = py_trees.visitors.DisplaySnapshotVisitor()

    behaviour_tree.visitors.append(debug_visitor)
    behaviour_tree.visitors.append(snapshot_visitor)

    behaviour_tree.add_post_tick_handler(functools.partial(logger, snapshot_visitor))

    behaviour_tree.setup(timeout=15)

    ####################
    # Tick Tock
    ####################
    if args.interactive:
        py_trees.console.read_single_keypress()
    while True:
        try:
            behaviour_tree.tick()
            if args.interactive:
                py_trees.console.read_single_keypress()
            else:
                time.sleep(0.5)
        except KeyboardInterrupt:
            break
    print("\n")

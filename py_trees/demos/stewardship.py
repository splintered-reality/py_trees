#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/splintered-reality/py_trees/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
.. argparse::
   :module: py_trees.demos.stewardship
   :func: command_line_argument_parser
   :prog: py-trees-demo-tree-stewardship

.. graphviz:: dot/demo-tree-stewardship.dot

.. image:: images/tree_stewardship.gif
"""

##############################################################################
# Imports
##############################################################################

import argparse
import py_trees
import sys
import time

import py_trees.console as console

##############################################################################
# Classes
##############################################################################


def description():
    content = "A demonstration of tree stewardship.\n\n"
    content += "A slightly less trivial tree that uses a simple stdout pre-tick handler\n"
    content += "and both the debug and snapshot visitors for logging and displaying\n"
    content += "the state of the tree.\n"
    content += "\n"
    content += "EVENTS\n"
    content += "\n"
    content += " -  3 : sequence switches from running to success\n"
    content += " -  4 : selector's first child flicks to success once only\n"
    content += " -  8 : the fallback idler kicks in as everything else fails\n"
    content += " - 14 : the first child kicks in again, aborting a running sequence behind it\n"
    content += "\n"
    if py_trees.console.has_colours:
        banner_line = console.green + "*" * 79 + "\n" + console.reset
        s = "\n"
        s += banner_line
        s += console.bold_white + "Trees".center(79) + "\n" + console.reset
        s += banner_line
        s += "\n"
        s += content
        s += "\n"
        s += banner_line
    else:
        s = content
    return s


def epilog():
    if py_trees.console.has_colours:
        return console.cyan + "And his noodly appendage reached forth to tickle the blessed...\n" + console.reset
    else:
        return None


def command_line_argument_parser():
    parser = argparse.ArgumentParser(description=description(),
                                     epilog=epilog(),
                                     formatter_class=argparse.RawDescriptionHelpFormatter,
                                     )
    group = parser.add_mutually_exclusive_group()
    group.add_argument('-r', '--render', action='store_true', help='render dot tree to file')
    group.add_argument(
        '--render-with-blackboard-variables',
        action='store_true',
        help='render dot tree to file with blackboard variables'
    )
    group.add_argument('-i', '--interactive', action='store_true', help='pause and wait for keypress at each tick')
    return parser


def pre_tick_handler(behaviour_tree):
    print("\n--------- Run %s ---------\n" % behaviour_tree.count)


class SuccessEveryN(py_trees.behaviours.SuccessEveryN):
    def __init__(self):
        super().__init__(name="EveryN", n=5)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key("count", access=py_trees.common.Access.WRITE)

    def update(self):
        status = super().update()
        self.blackboard.count = self.count
        return status


class PeriodicSuccess(py_trees.behaviours.Periodic):
    def __init__(self):
        super().__init__(name="Periodic", n=3)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key("period", access=py_trees.common.Access.WRITE)

    def update(self):
        status = super().update()
        self.blackboard.period = self.period
        return status


class Finisher(py_trees.behaviour.Behaviour):
    def __init__(self):
        super().__init__(name="Finisher")
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key("count", access=py_trees.common.Access.READ)
        self.blackboard.register_key("period", access=py_trees.common.Access.READ)

    def update(self):
        print(console.green + "---------------------------" + console.reset)
        print(console.bold + "        Finisher" + console.reset)
        print(console.green + "  Count : {}".format(self.blackboard.count) + console.reset)
        print(console.green + "  Period: {}".format(self.blackboard.period) + console.reset)
        print(console.green + "---------------------------" + console.reset)
        return py_trees.common.Status.SUCCESS


def create_tree():
    every_n_success = SuccessEveryN()
    sequence = py_trees.composites.Sequence(name="Sequence")
    guard = py_trees.behaviours.Success("Guard")
    periodic_success = PeriodicSuccess()
    finisher = Finisher()
    sequence.add_child(guard)
    sequence.add_child(periodic_success)
    sequence.add_child(finisher)
    idle = py_trees.behaviours.Success("Idle")
    root = py_trees.composites.Selector(name="Demo Tree")
    root.add_child(every_n_success)
    root.add_child(sequence)
    root.add_child(idle)
    return root


##############################################################################
# Main
##############################################################################

def main():
    """
    Entry point for the demo script.
    """
    args = command_line_argument_parser().parse_args()
    py_trees.logging.level = py_trees.logging.Level.DEBUG
    tree = create_tree()
    print(description())

    ####################
    # Rendering
    ####################
    if args.render:
        py_trees.display.render_dot_tree(tree)
        sys.exit()

    if args.render_with_blackboard_variables:
        py_trees.display.render_dot_tree(tree, with_blackboard_variables=True)
        sys.exit()

    ####################
    # Tree Stewardship
    ####################
    py_trees.blackboard.Blackboard.enable_activity_stream(100)
    behaviour_tree = py_trees.trees.BehaviourTree(tree)
    behaviour_tree.add_pre_tick_handler(pre_tick_handler)
    behaviour_tree.visitors.append(py_trees.visitors.DebugVisitor())
    behaviour_tree.visitors.append(
        py_trees.visitors.DisplaySnapshotVisitor(
            display_blackboard=True,
            display_activity_stream=True)
    )
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

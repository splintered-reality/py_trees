#!/usr/bin/env python
#
# License: Yujin
#
##############################################################################
# Description
##############################################################################

"""
.. module:: ar_markers
   :platform: Unix
   :synopsis: Working with the ar marker node

Oh my spaghettified magnificence,
Bless my noggin with a tickle from your noodly appendages!

"""

##############################################################################
# Imports
##############################################################################

import argparse
import py_trees
import rocon_console.console as console
import rospy

##############################################################################
# Methods
##############################################################################


def create_arg_parser(show_description, show_usage):
    """
    Generate a basic arg parser with standard py_trees rendering, visibility
    and debugging options.

    :param func show_description: string generating function for description
    :param func show_usage: string generating function for usage
    """
    parser = argparse.ArgumentParser(description=show_description(),
                                     usage=show_usage(),
                                     epilog="And his noodly appendage reached forth to tickle the blessed...",
                                     formatter_class=argparse.ArgumentDefaultsHelpFormatter
                                     )
    parser.add_argument('-r', '--render', action='store_true', help='render the graph to dot/png/svg.')
    parser.add_argument('-v', '--visibility-level',
                        action='store',
                        default="detail",
                        choices=py_trees.common.visibility_level_strings,
                        help='visibility level for blackboxes when using -r'
                        )
    parser.add_argument('-d', '--debug', action='store_true', help='debug level tree logging.')
    return parser

##############################################################################
# Classes
##############################################################################


class Demo(object):
    def __init__(self, root, loop=False):
        """
        :param py_trees.behaviour.Behaviour root:
        :param bool loop: loop around, never quit.
        """
        self.root = root
        self.logger = py_trees.logging.get_logger("RePark")
        self.tree = py_trees.ROSBehaviourTree(self.root)
        self.loop = loop

    def setup(self, timeout):
        rospy.on_shutdown(self.shutdown)
        return self.tree.setup(timeout)

    ##############################################################################
    # Tick Tock
    ##############################################################################

    def tick_tock(self):
        if self.loop:
            self.tree.tick_tock(
                sleep_ms=500,
                number_of_iterations=py_trees.trees.CONTINUOUS_TICK_TOCK,
                pre_tick_handler=self.pre_tick_handler
            )
        else:
            loop_around = rospy.Rate(2)
            self.tree.visitors.append(py_trees.trees.DebugVisitor())
            while not self.root.status == py_trees.Status.SUCCESS\
                    and not self.root.status == py_trees.Status.FAILURE\
                    and not rospy.is_shutdown():
                self.tree.tick(pre_tick_handler=self.pre_tick_handler)
                loop_around.sleep()

    def pre_tick_handler(self, behaviour_tree):
        self.logger.debug("")
        self.logger.debug("{:-^30}".format(" Run %s " % behaviour_tree.count))
        self.logger.debug("")

    def shutdown(self):
        print(console.bold + "\n******************** Shutdown ********************\n" + console.reset)
        self.tree.destroy()  # destroy the tree on shutdown to stop the behaviour
        self.tree.interrupt()

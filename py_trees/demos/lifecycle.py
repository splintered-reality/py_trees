#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/stonier/py_trees/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
.. argparse::
   :module: py_trees.demos.lifecycle
   :func: command_line_argument_parser
   :prog: py-trees-demo-behaviour-lifecycle

.. image:: images/lifecycle.gif
"""

##############################################################################
# Imports
##############################################################################

import argparse
import py_trees
import time

import py_trees.console as console

##############################################################################
# Classes
##############################################################################


def description():
    content = "Demonstrates a typical day in the life of a behaviour.\n\n"
    content += "This behaviour will count from 1 to 3 and then reset and repeat. As it does\n"
    content += "so, it logs and displays the methods as they are called - construction, setup,\n"
    content += "initialisation, ticking and termination.\n"
    if py_trees.console.has_colours:
        banner_line = console.green + "*" * 79 + "\n" + console.reset
        s = "\n"
        s += banner_line
        s += console.bold_white + "Behaviour Lifecycle".center(79) + "\n" + console.reset
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
    return argparse.ArgumentParser(description=description(),
                                   epilog=epilog(),
                                   formatter_class=argparse.RawDescriptionHelpFormatter,
                                   )


class Counter(py_trees.behaviour.Behaviour):
    """
    Simple counting behaviour that facilitates the demonstration of a behaviour in
    the demo behaviours lifecycle program.

    * Increments a counter from zero at each tick
    * Finishes with success if the counter reaches three
    * Resets the counter in the initialise() method.
    """
    def __init__(self, name="Counter"):
        """
        Default construction.
        """
        super(Counter, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

    def setup(self, unused_timeout=15):
        """
        No delayed initialisation required for this example.
        """
        self.logger.debug("%s.setup()" % (self.__class__.__name__))
        return True

    def initialise(self):
        """
        Reset a counter variable.
        """
        self.logger.debug("%s.initialise()" % (self.__class__.__name__))
        self.counter = 0

    def update(self):
        """
        Increment the counter and decide upon a new status result for the behaviour.
        """
        self.counter += 1
        new_status = py_trees.common.Status.SUCCESS if self.counter == 3 else py_trees.common.Status.RUNNING
        if new_status == py_trees.common.Status.SUCCESS:
            self.feedback_message = "counting...{0} - phew, thats enough for today".format(self.counter)
        else:
            self.feedback_message = "still counting"
        self.logger.debug("%s.update()[%s->%s][%s]" % (self.__class__.__name__, self.status, new_status, self.feedback_message))
        return new_status

    def terminate(self, new_status):
        """
        Nothing to clean up in this example.
        """
        self.logger.debug("%s.terminate()[%s->%s]" % (self.__class__.__name__, self.status, new_status))


##############################################################################
# Main
##############################################################################

def main():
    """
    Entry point for the demo script.
    """
    command_line_argument_parser().parse_args()

    print(description())

    py_trees.logging.level = py_trees.logging.Level.DEBUG

    counter = Counter()
    counter.setup()
    try:
        for unused_i in range(0, 7):
            counter.tick_once()
            time.sleep(0.5)
        print("\n")
    except KeyboardInterrupt:
        print("")
        pass

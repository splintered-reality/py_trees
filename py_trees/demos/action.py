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
   :module: py_trees.demos.action
   :func: command_line_argument_parser
   :prog: py-trees-demo-action-behaviour

.. image:: images/action.gif
"""

##############################################################################
# Imports
##############################################################################

import argparse
import atexit
import multiprocessing
import py_trees
import py_trees.common
import time

import py_trees.console as console

##############################################################################
# Classes
##############################################################################


def description():
    content = "Demonstrates the characteristics of a typical 'action' behaviour.\n"
    content += "\n"
    content += "* Mocks an external process and connects to it in the setup() method\n"
    content += "* Kickstarts new goals with the external process in the initialise() method\n"
    content += "* Monitors the ongoing goal status in the update() method\n"
    content += "* Determines RUNNING/SUCCESS pending feedback from the external process\n"

    if py_trees.console.has_colours:
        banner_line = console.green + "*" * 79 + "\n" + console.reset
        s = "\n"
        s += banner_line
        s += console.bold_white + "Action Behaviour".center(79) + "\n" + console.reset
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


def planning(pipe_connection):
    """
    Emulates an external process which might accept long running planning jobs.
    """
    idle = True
    percentage_complete = 0
    try:
        while(True):
            if pipe_connection.poll():
                pipe_connection.recv()
                percentage_complete = 0
                idle = False
            if not idle:
                percentage_complete += 10
                pipe_connection.send([percentage_complete])
                if percentage_complete == 100:
                    idle = True
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass


class Action(py_trees.behaviour.Behaviour):
    """
    Connects to a subprocess to initiate a goal, and monitors the progress
    of that goal at each tick until the goal is completed, at which time
    the behaviour itself returns with success or failure (depending on
    success or failure of the goal itself).

    This is typical of a behaviour that is connected to an external process
    responsible for driving hardware, conducting a plan, or a long running
    processing pipeline (e.g. planning/vision).

    Key point - this behaviour itself should not be doing any work!
    """
    def __init__(self, name="Action"):
        """
        Default construction.
        """
        super(Action, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

    def setup(self, unused_timeout=15):
        """
        No delayed initialisation required for this example.
        """
        self.logger.debug("%s.setup()->connections to an external process" % (self.__class__.__name__))
        self.parent_connection, self.child_connection = multiprocessing.Pipe()
        self.planning = multiprocessing.Process(target=planning, args=(self.child_connection,))
        atexit.register(self.planning.terminate)
        self.planning.start()
        return True

    def initialise(self):
        """
        Reset a counter variable.
        """
        self.logger.debug("%s.initialise()->sending new goal" % (self.__class__.__name__))
        self.parent_connection.send(['new goal'])
        self.percentage_completion = 0

    def update(self):
        """
        Increment the counter and decide upon a new status result for the behaviour.
        """
        new_status = py_trees.common.Status.RUNNING
        if self.parent_connection.poll():
            self.percentage_completion = self.parent_connection.recv().pop()
            if self.percentage_completion == 100:
                new_status = py_trees.common.Status.SUCCESS
        if new_status == py_trees.common.Status.SUCCESS:
            self.feedback_message = "Processing finished"
            self.logger.debug("%s.update()[%s->%s][%s]" % (self.__class__.__name__, self.status, new_status, self.feedback_message))
        else:
            self.feedback_message = "{0}%".format(self.percentage_completion)
            self.logger.debug("%s.update()[%s][%s]" % (self.__class__.__name__, self.status, self.feedback_message))
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

    action = Action()
    action.setup()
    try:
        for unused_i in range(0, 12):
            action.tick_once()
            time.sleep(0.5)
        print("\n")
    except KeyboardInterrupt:
        pass

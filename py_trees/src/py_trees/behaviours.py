#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/yujinrobot/gopher_crazy_hospital/license/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
.. module:: behaviours
   :platform: Unix
   :synopsis: Behaviour templates for use in behaviour trees.

This module defines the interface for behaviours to be used in py_trees.

----

"""

##############################################################################
# Imports
##############################################################################

from .common import Status

##############################################################################
# Behaviour
##############################################################################


class Behaviour(object):
    """ A node in a behavior tree that uses coroutines in its tick function """

    def __init__(self, name="", *args, **kwargs):
        self.name = name
        self.status = Status.INVALID
        self.iterator = self.tick()
        # Behaviours on their own are leaf nodes, forcibly require them to have
        # no children, but include this variable so it is easy to stop at a leaf
        # while parsing down the tree.
        self.children = []

    ############################################
    # User Defined Functions (virtual)
    ############################################

    def initialise(self):
        pass

    def terminate(self, new_status):
        """
        :param Status new_status: compare with current status for decision logic.

        User defined terminate (cleanup) function. For comparison tests, it is
        often useful to check if the current status is Status.RUNNING and
        the new status is different to trigger appropriate cleanup actions.
        """
        pass

    def update(self):
        return Status.INVALID

    ############################################
    # Workers
    ############################################

    def tick(self):
        """
        The update mechanism for the behaviour. Customisation goes
        into the update() function.

        @return Status : resulting state after the tick is completed
        """
        while True:
            if self.status != Status.RUNNING:
                self.initialise()
            # don't set self.status yet, terminate() may need to check what the current state is first
            new_status = self.update()
            if new_status != Status.RUNNING:
                self.abort(new_status)
            self.status = new_status
            yield self.status

    def abort(self, new_status=Status.INVALID):
        """
        :param Status new_status: set the status to this once done.

        This calls the user defined terminate() method and also resets the
        generator. The specified status can be used to compare with the
        current status field for decision making within the terminate function
        itself.
        """
        self.terminate(new_status)
        self.status = new_status
        self.iterator = self.tick()

    ############################################
    # Status
    ############################################

    def get_status(self):
        return self.status

    def set_status(self, s):
        self.status = s

    def is_running(self):
        return self.status == Status.RUNNING

    def is_terminated(self):
        return (self.status == Status.SUCCESS) or (self.status == Status.FAILURE)

    def announce(self):
        print("Executing behaviour " + str(self.name))

    # These next two functions allow us to use the 'with' syntax
    def __enter__(self):
        return self.name

    def __exit__(self, exc_type, exc_val, exc_tb):
        if exc_type is not None:
            return False
        return True

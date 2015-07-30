#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/yujinrobot/gopher_crazy_hospital/license/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
.. module:: composites
   :platform: Unix
   :synopsis: This module defines standard composites for use in py_trees.

The most important ones are the selector and sequence, but some more esoteric
versions are also included here.

----

"""

##############################################################################
# Imports
##############################################################################

import itertools

from .behaviours import Behaviour
from .common import Status

##############################################################################
# Composites
##############################################################################


class Composite(Behaviour):
    """ A node in a behavior tree that composites behaviours """
    def __init__(self, children=[], name="", *args, **kwargs):
        super(Composite, self).__init__(name, *args, **kwargs)
        self.children = children if children is not None else []

    ############################################
    # Worker Overrides
    ############################################

    def abort(self, new_status=Status.INVALID):
        for child in self.children:
            child.abort(Status.INVALID)
        self.iterator = self.tick()

    ############################################
    # Children
    ############################################

    def add_child(self, child):
        self.children.append(child)

    def remove_child(self, child):
        self.children.remove(child)

    def prepend_child(self, c):
        self.children.insert(0, c)

    def insert_child(self, c, i):
        self.children.insert(i, c)

##############################################################################
# Selector
##############################################################################


class Selector(Composite):
    """
    Runs all of its child behaviours in sequence until one succeeds.
    If a node after than the one selected was previously running, then call
    abort() on it.
    """

    def __init__(self, children=[], name="Selector", *args, **kwargs):
        super(Selector, self).__init__(children, name, *args, **kwargs)
        self.current = None

    def update(self):
        print("  %s : iterating" % self.name)
        for child in self.children:
            status = next(child.iterator)
            if (status == Status.RUNNING) or (status == Status.SUCCESS):
                self.current = child
                return status
        return Status.FAILURE

    def tick(self):
        while True:
            previous = self.current
            self.status = self.update()
            if (previous is not None) and (previous != self.current):
                previous.abort(Status.INVALID)
            yield self.status

    def abort(self, new_status=Status.INVALID):
        self.current_child = None
        Composite.abort(self, new_status)

##############################################################################
# Sequence
##############################################################################


class Sequence(Composite):
    """
    Runs all of its children in sequence until all succeed
    """

    def __init__(self, children=[], name="Sequence", *args, **kwargs):
        super(Sequence, self).__init__(children, name, *args, **kwargs)
        self.current_child = None
        self.current_index = 0

    def initialise(self):
        """
        If it isn't already running, then start the sequence from the
        beginning.
        """
        self.current_child = None
        self.current_index = 0

    def update(self):
        print("  %s : iterating" % self.name)
        for child in itertools.islice(self.children, self.current_index, None):
            status = next(child.iterator)
            if status != Status.SUCCESS:
                # don't worry about self.current_index, it will reset naturally via tick() & initialise() if necessary
                return status
            self.current_index += 1
        return Status.SUCCESS

    def abort(self, new_status=Status.INVALID):
        self.current_child = None
        Composite.abort(self, new_status)

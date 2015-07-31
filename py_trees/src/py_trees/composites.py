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
        print("  %s [Composite::abort()]" % self.name)
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
        self.current_child = None

    def update(self):
        print("  %s [Selector::update()]" % self.name)
        for child in self.children:
            status = next(child.iterator)
            if (status == Status.RUNNING) or (status == Status.SUCCESS):
                self.current = child
                return status
        return Status.FAILURE

    def tick(self):
        print("  %s [Selector::tick()]" % self.name)
        previous = self.current_child
        for child in self.children:
            print("    Selector Child: %s" % child.name)
            for node in child.tick():
                yield node
                if node.status == Status.RUNNING or node.status == Status.SUCCESS:
                    self.current_child = child
                    if self.current_child is not None:
                        print("    Selector Current Child : %s" % self.current_child.name)
                    if previous is not None:
                        print("    Selector Previous Child: %s" % previous.name)
                    self.status = node.status
                    if (previous is not None) and (previous != self.current_child) and (previous.status == Status.RUNNING):
                        print("***** Aborting old selection *****")
                        print("    Previous: %s" % previous.name)
                        previous.abort(Status.INVALID)
                    yield self
                    return
        self.status = Status.FAILURE
        yield self

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
        self.current_index = 0

    def initialise(self):
        """
        If it isn't already running, then start the sequence from the
        beginning.
        """
        print("  %s [Sequence::initialise()]" % self.name)
        self.current_child = None
        self.current_index = 0

    def tick(self):
        print("  %s [Sequence::tick()]" % self.name)
        for child in itertools.islice(self.children, self.current_index, None):
            for node in child.tick():
                yield node
                print("Next in line")
                if node.status != Status.SUCCESS:
                    self.status = node.status
                    yield self
                    return
            self.current_index += 1
        self.status = Status.SUCCESS
        yield self

    def abort(self, new_status=Status.INVALID):
        print("  %s [Sequence::abort()]" % self.name)
        self.current_index = 0
        Composite.abort(self, new_status)

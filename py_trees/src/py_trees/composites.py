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
import logging

from .behaviours import Behaviour
from .common import Status

##############################################################################
# Composites
##############################################################################


class Composite(Behaviour):
    """ A node in a behavior tree that composites behaviours """
    def __init__(self, name="", children=None, *args, **kwargs):
        super(Composite, self).__init__(name, *args, **kwargs)
        self.children = children if children is not None else []
        self.logger = logging.getLogger("py_trees.Composite")

    ############################################
    # Worker Overrides
    ############################################

    def abort(self, new_status=Status.INVALID):
        self.logger.debug("  %s [abort()]" % self.name)
        for child in self.children:
            child.abort(Status.INVALID)
        self.status = new_status
        self.iterator = self.tick()

    ############################################
    # Children
    ############################################

    def add_child(self, child):
        """
        Adds a child.

        :param child: child to add to the tree
        :type child: an instance (or descendant) of py_trees.Behaviour
        :return: a unique id for this child (store and use to form remove requests)
        """
        self.children.append(child)
        return child.id

    def remove_child(self, child):
        self.children.remove(child)

    def remove_child_by_id(self, child_id):
        child = next((c for c in self.children if c.id == child_id), None)
        if child is not None:
            self.children.remove(child)
        else:
            raise IndexError('child was not found with the specified id [%s]' % child_id)

    def prepend_child(self, child):
        self.children.insert(0, child)
        return child.id

    def insert_child(self, child, index):
        self.children.insert(index, child)
        return child.id

##############################################################################
# Selector
##############################################################################


class Selector(Composite):
    """
    Runs all of its child behaviours in sequence until one succeeds.
    If a node after than the one selected was previously running, then call
    abort() on it.
    """

    def __init__(self, name="Selector", children=None, *args, **kwargs):
        super(Selector, self).__init__(name, children, *args, **kwargs)
        self.current_child = None
        self.logger = logging.getLogger("py_trees.Selector")

    def update(self):
        self.logger.debug("  %s [update()]" % self.name)
        for child in self.children:
            status = next(child.iterator)
            if (status == Status.RUNNING) or (status == Status.SUCCESS):
                self.current = child
                return status
        return Status.FAILURE

    def tick(self):
        self.logger.debug("  %s [tick()]" % self.name)
        previous = self.current_child
        for child in self.children:
            for node in child.tick():
                yield node
                if node is child:
                    if node.status == Status.RUNNING or node.status == Status.SUCCESS:
                        self.current_child = child
                        self.status = node.status
                        if (previous is not None) and (previous != self.current_child) and (previous.status == Status.RUNNING):
                            previous.abort(Status.INVALID)
                        yield self
                        return
        self.status = Status.FAILURE
        yield self

    def abort(self, new_status=Status.INVALID):
        self.current_child = None
        Composite.abort(self, new_status)

    def __repr__(self):
        s = "Name       : %s\n" % self.name
        s += "  Status  : %s\n" % self.status
        s += "  Current : %s\n" % (self.current_child.name if self.current_child is not None else "none")
        s += "  Children: %s\n" % [child.name for child in self.children]

##############################################################################
# Sequence
##############################################################################


class Sequence(Composite):
    """
    Runs all of its children in sequence until all succeed
    """

    def __init__(self, children=None, name="Sequence", *args, **kwargs):
        super(Sequence, self).__init__(name, children, *args, **kwargs)
        self.current_index = 0
        self.logger = logging.getLogger("py_trees.Sequence")

    def initialise(self):
        """
        If it isn't already running, then start the sequence from the
        beginning.
        """
        self.logger.debug("  %s [initialise()]" % self.name)
        self.current_child = None
        self.current_index = 0

    def tick(self):
        self.logger.debug("  %s [tick()]" % self.name)
        for child in itertools.islice(self.children, self.current_index, None):
            for node in child.tick():
                yield node
                if node is child and node.status != Status.SUCCESS:
                    self.status = node.status
                    yield self
                    return
            self.current_index += 1
        # At this point, all children are happy with their SUCCESS, so we should be happy with SUCCESS too
        self.abort(Status.SUCCESS)
        yield self

    def abort(self, new_status=Status.INVALID):
        self.logger.debug("  %s [abort()][%s]" % (self.name, self.status))
        self.current_index = 0
        self.status = new_status
        # all of the children will already have aborted themselves, so no need to call abort on them again
        # Composite.abort(self, new_status)

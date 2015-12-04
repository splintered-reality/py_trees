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

from __future__ import absolute_import

import itertools

from . import logging
from .behaviours import Behaviour
from .common import Status
from . import meta

##############################################################################
# Composites
##############################################################################


class Composite(Behaviour):
    """ A node in a behavior tree that composites behaviours """
    def __init__(self, name="", children=None, *args, **kwargs):
        super(Composite, self).__init__(name, *args, **kwargs)
        if children is not None:
            map(self.add_child, children)
        else:
            self.children = []
        self.logger = logging.get_logger("Composite")


    ############################################
    # Worker Overrides
    ############################################

    def stop(self, new_status=Status.INVALID):
        """
        This updates all the children. We generally have two use cases here - whenever the composite has
        gone to a recognised state (i.e. FAILURE or SUCCESS), or when a higher level parent calls on
        it to truly stop (INVALID). We only really want to update everything below if we are in
        the second use case. The first use case will subehaviours will have already handled themselves.
        """
        self.logger.debug("  %s [Composite.stop()][%s->%s]" % (self.name, self.status, new_status))
        if new_status == Status.INVALID:
            for child in self.children:
                child.stop(new_status)
        self.status = new_status
        self.iterator = self.tick()

    def tip(self):
        """Recursive function to extract the last running node of the tree. Returns the
        tip function of the current child of this composite.

        """
        return self.current_child.tip() if self.current_child is not None else None

    ############################################
    # Children
    ############################################

    def add_child(self, child):
        """
        Adds a child.

        :param child: child to add to the tree
        :type child: instance or descendant of :class:`Behaviour <py_trees.behaviours.Behaviour>`
        :return: a unique id for this child (store and use to form remove requests)
        """
        self.children.append(child)
        child.parent = self
        return child.id

    def remove_child(self, child):
        """
        Remove the child behaviour from this composite.

        :param child: child to remove from the tree
        :type child: instance or descendant of :class:`Behaviour <py_trees.behaviours.Behaviour>`

        .. todo:: error handling for when child is not in this list (what error is thrown?)
        """
        if child.status == Status.RUNNING:
            child.stop(Status.INVALID)
        index = self.children.index(child)
        self.children.remove(child)
        return index

    def replace_child(self, child, replacement):
        """
        Replace the child behaviour.

        :param child: child to remove from the tree
        :type child: instance or descendant of :class:`Behaviour <py_trees.behaviours.Behaviour>`
        :param replacement: child to insert
        :type replacement: instance or descendant of :class:`Behaviour <py_trees.behaviours.Behaviour>`
        """
        if child.status == Status.RUNNING:
            child.stop(Status.INVALID)
        index = self.children.index(child)
        self.logger.debug("  %s [replace_child()][%s->%s]" % (self.name, child.name, replacement.name))
        self.children[index] = replacement

    def remove_child_by_id(self, child_id):
        child = next((c for c in self.children if c.id == child_id), None)
        if child is not None:
            if child.status == Status.RUNNING:
                child.stop(Status.INVALID)
            self.children.remove(child)
        else:
            raise IndexError('child was not found with the specified id [%s]' % child_id)

    def prepend_child(self, child):
        self.children.insert(0, child)
        child.parent = self
        return child.id

    def insert_child(self, child, index):
        self.children.insert(index, child)
        child.parent = self
        return child.id

##############################################################################
# Selector
##############################################################################


class Selector(Composite):
    """
    Runs all of its child behaviours in sequence until one succeeds.
    If a node after than the one selected was previously running, then call
    stop() on it.
    """

    def __init__(self, name="Selector", children=None, *args, **kwargs):
        super(Selector, self).__init__(name, children, *args, **kwargs)
        self.current_child = None
        self.logger = logging.get_logger("Selector ")

    def initialise(self):
        self.current_child = None

    def update(self):
        self.logger.debug("  %s [update()]" % self.name)
        for child in self.children:
            status = next(child.iterator)
            if (status == Status.RUNNING) or (status == Status.SUCCESS):
                self.current = child
                return status
        return Status.FAILURE

    def tick(self):
        # Required behaviour for *all* behaviours and composites is
        # for tick() to check if it isn't running and initialise
        #
        # Selector doesn't actually have anything to do though, but just
        # in case someone inherits the class and overrides initialise()
        # but not tick()
        if self.status != Status.RUNNING:
            self.initialise()
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
                            previous.stop(Status.INVALID)
                        yield self
                        return
        self.status = Status.FAILURE
        yield self

    def stop(self, new_status=Status.INVALID):
        # retain information about the last running child if the new status is
        # SUCCESS or FAILURE
        if new_status == Status.INVALID:
            self.current_child = None
        Composite.stop(self, new_status)

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

    def __init__(self, name="Sequence", children=None, *args, **kwargs):
        super(Sequence, self).__init__(name, children, *args, **kwargs)
        self.current_index = -1 # -1 indicates uninitialised
        self.logger = logging.get_logger("Sequence ")

    def initialise(self):
        """
        If it isn't already running, then start the sequence from the
        beginning.
        """
        self.logger.debug("  %s [initialise()]" % self.name)
        self.current_index = 0

    def tick(self):
        if self.status != Status.RUNNING:
            self.initialise()
        self.logger.debug("  %s [tick()]" % self.name)
        for child in itertools.islice(self.children, self.current_index, None):
            for node in child.tick():
                yield node
                if node is child and node.status != Status.SUCCESS:
                    self.status = node.status
                    yield self
                    return
            self.current_index += 1
        # At this point, all children are happy with their SUCCESS, so we should be happy too
        self.current_index -= 1 # went off the end of the list if we got to here
        self.stop(Status.SUCCESS)
        yield self

    @property
    def current_child(self):
        if self.current_index == -1:
            return None
        return self.children[self.current_index] if self.children else None

    def stop(self, new_status=Status.INVALID):
        self.logger.debug("  %s [stop()][%s->%s]" % (self.name, self.status, new_status))
        # retain information about the last running child if the new status is
        # SUCCESS or FAILURE
        if new_status == Status.INVALID:
            self.current_index = -1
        self.status = new_status
        Composite.stop(self, new_status)


@meta.oneshot
class OneshotSequence(Sequence):
    pass

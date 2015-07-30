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

    def abort(self):
        for child in self.children:
            child.abort()
        self.terminate()
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
    """

    def __init__(self, children=[], name="Selector", *args, **kwargs):
        super(Selector, self).__init__(children, name, *args, **kwargs)
        self.current_child = None

    def tick(self):
        for child in self.children:
            self.current_child = child
            for status in child.iterator:
                if status == Status.RUNNING:
                    yield status
                elif status == Status.FAILURE:
                    yield Status.RUNNING
                else:
                    yield status
                    return
        yield Status.FAILURE

    def abort(self):
        self.current_child = None
        Composite.abort(self)

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

    def tick(self):
        for child in self.children:
            self.current_child = child
            for status in child.iterator:
                if status == Status.RUNNING:
                    yield status
                elif status == Status.FAILURE:
                    yield Status.FAILURE
                    return
                else:
                    yield Status.RUNNING
        yield Status.SUCCESS

    def abort(self):
        self.current_child = None
        Composite.abort(self)

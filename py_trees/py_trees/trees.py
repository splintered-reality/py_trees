#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/stonier/py_trees_suite/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
Tree managers - they make your life easier!
"""

##############################################################################
# Imports
##############################################################################

import time

from . import composites

from .behaviours import Behaviour

CONTINUOUS_TICK_TOCK = -1

##############################################################################
# Trees
##############################################################################


class BehaviourTree(object):
    """
    Grow, water, prune your behaviour tree.

    :ivar int count: number of times the tree has been ticked.
    :ivar root: root node of the tree.
    :vartype root: instance or descendant of :class:`Behaviour <py_trees.behaviours.Behaviour>`
    :ivar visitors: list of entities that visit the iterated parts of the tree when it ticks.
    :vartype visitors: list of classes that implement an initialise() and run(py_trees.Behaviour) method.
    :ivar pre_tick_handlers: methods that run before the entire tree is ticked
    :vartype pre_tick_handlers: list of methods that take this instance as an arg
    :ivar post_tick_handlers: methods that run after the entire tree is ticked
    :vartype post_tick_handlers: list of methods that take this instance as an arg
    :ivar interrupt_tick_tocking: interrupt tick-tock if it is a tick-tocking.
    :vartype interrupt_tick_tocking: bool
    """
    def __init__(self, root):
        """
        Initialise the tree with a root.

        :param root: root node of the tree.
        :type root: instance or descendant of :class:`Behaviour <py_trees.behaviours.Behaviour>`
        :raises AssertionError: if incoming root variable is not the correct type.
        """
        self.count = 0
        assert root is not None, "root node must not be 'None'"
        assert isinstance(root, Behaviour), "root node must be an instance of or descendant of pytrees.Behaviour"
        self.root = root
        self.visitors = []
        self.pre_tick_handlers = []
        self.post_tick_handlers = []
        self.interrupt_tick_tocking = False
        self.tree_update_handler = None  # child classes can utilise this one

    def add_pre_tick_handler(self, handler):
        self.pre_tick_handlers.append(handler)

    def add_post_tick_handler(self, handler):
        self.post_tick_handlers.append(handler)

    def prune_subtree(self, unique_id):
        """
        :param uuid.UUID unique_id: unique id of the subtree root
        :raises AssertionError: if unique id is the behaviour tree's root node id
        :return: success or failure of the pruning
        :rtype: bool
        """
        assert self.root.id != unique_id, "may not prune the root node"
        for child in self.root.iterate():
            if child.id == unique_id:
                parent = child.parent
                if parent is not None:
                    parent.remove_child(child)
                    if self.tree_update_handler is not None:
                        self.tree_update_handler(self.root)
                    return True
        return False

    def insert_subtree(self, child, unique_id, index):
        """
        Insert subtree as a child of the specified parent.

        :param uuid.UUID unique_id: id of the parent container (usually Selector or Sequence)
        :param int index: insert the child at this index, pushing all children after it back one.
        :return: False if parent node was not found, True if inserted

        .. todo::

           Could use better, more informative error handling here. Especially if the insertion
           has its own error handling (e.g. index out of range).

           Could also use a different api that relies on the id
           of the sibling node it should be inserted before/after.
        """
        for node in self.root.iterate():
            if node.id == unique_id:
                assert isinstance(node, composites.Composite), "parent must be a Composite behaviour."
                node.insert_child(child, index)
                if self.tree_update_handler is not None:
                    self.tree_update_handler(self.root)
                return True
        return False

    def replace_subtree(self, unique_id, subtree):
        """
        Replace the subtree with the specified id for the new subtree.

        This is a common pattern where we'd like to swap out a whole sub-behaviour for another one.

        :param uuid.UUID unique_id: unique id of the subtree root
        :param subtree: incoming subtree to replace the old one
        :type subtree: instance or descendant of :class:`Behaviour <py_trees.behaviours.Behaviour>`
        :raises AssertionError: if unique id is the behaviour tree's root node id
        :return: success or failure of the replacement
        :rtype: bool
        """
        assert self.root.id != unique_id, "may not replace the root node"
        for child in self.root.iterate():
            if child.id == unique_id:
                parent = child.parent
                if parent is not None:
                    parent.replace_child(child, subtree)
                    if self.tree_update_handler is not None:
                        self.tree_update_handler(self.root)
                    return True
        return False

    def setup(self, timeout):
        """
        Calls setup on the root of the tree. This should then percolate down into the tree itself
        via the :py:meth:`~py_trees.composites.Composite.setup` function in each composited behaviour.

        :returns: success or failure of the setup operation
        """
        return self.root.setup(timeout)

    def tick(self, pre_tick_handler=None, post_tick_handler=None):
        """
        Tick over the tree just once.

        :param pre_tick_visitor: visitor that runs on this class instance before the tick
        :type pre_tick_visitor: any class with a run(py_trees.BehaviourTree) method
        :param post_tick_visitor: visitor that runs on this class instance before the tick
        :type post_tick_visitor: any class with a run(py_trees.BehaviourTree) method
        """
        # pre
        for handler in self.pre_tick_handlers:
            handler(self)
        if pre_tick_handler is not None:
            pre_tick_handler(self)
        for visitor in self.visitors:
            visitor.initialise()
        # tick
        for node in self.root.tick():
            for visitor in [visitor for visitor in self.visitors if not visitor.full]:
                node.visit(visitor)

        for node in self.root.iterate():
            for visitor in [visitor for visitor in self.visitors if visitor.full]:
                node.visit(visitor)

        # post
        for handler in self.post_tick_handlers:
            handler(self)
        if post_tick_handler is not None:
            post_tick_handler(self)
        self.count += 1

    def tick_tock(self, sleep_ms, number_of_iterations=CONTINUOUS_TICK_TOCK, pre_tick_handler=None, post_tick_handler=None):
        """
        Tick continuously with a sleep interval as specified.

        :param float sleep_ms: sleep this much between ticks.
        :param int number_of_iterations: number of tick-tocks (-1 for infinite)
        :param pre_tick_handler: visitor that runs on this class instance before the tick
        :type pre_tick_handler: any function or class with a __call__ method taking a py_trees.BehaviourTree arg
        :param post_tick_handler: visitor that runs on this class instance before the tick
        :type post_tick_handler: any function or class with a __call__ method taking a py_trees.BehaviourTree arg
        """
        tick_tocks = 0
        while not self.interrupt_tick_tocking and (tick_tocks < number_of_iterations or number_of_iterations == CONTINUOUS_TICK_TOCK):
            self.tick(pre_tick_handler, post_tick_handler)
            try:
                time.sleep(sleep_ms / 1000.0)
            except KeyboardInterrupt:
                break
            tick_tocks += 1
        self.interrupt_tick_tocking = False

    def interrupt(self):
        """
        Interrupt tick-tock if it is tick-tocking.
        """
        self.interrupt_tick_tocking = True

    def destroy(self):
        """
        Destroy the tree by stopping the root node
        """
        self.root.stop()

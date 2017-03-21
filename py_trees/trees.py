#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/stonier/py_trees/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
While a graph of connected behaviours and composites form a tree in their own right
(i.e. it can be initialised and ticked), it is usually convenient to wrap your tree
in another class to take care of alot of the housework and provide some extra bells
and whistles that make your tree flourish.

.. image:: images/yggdrasil.jpg
   :width: 200px
   :align: center

This package provides a default reference implementation that is directly usable, but
can also be easily used as inspiration for your own tree custodians.
"""

##############################################################################
# Imports
##############################################################################

import time

from . import behaviour
from . import composites

CONTINUOUS_TICK_TOCK = -1

##############################################################################
# Trees
##############################################################################


class BehaviourTree(object):
    """
    Grow, water, prune your behaviour tree with this, the default reference
    implementation. It features a few enhancements to provide richer logging,
    introspection and dynamic management of the tree itself:

    * Pre and post tick handlers to execute code automatically before and after a tick
    * Visitor access to the parts of the tree that were traversed in a tick
    * Subtree pruning and insertion operations
    * Continuous tick-tock support

    .. seealso:: The :ref:`py-trees-demo-tree-stewardship-program` program demonstrates the above features.

    Args:
        root (:class:`~py_trees.behaviour.Behaviour`): root node of the tree

    Attributes:
        count (:obj:`int`): number of times the tree has been ticked.
        root (:class:`~py_trees.behaviour.Behaviour`): root node of the tree
        visitors ([:mod:`~py_trees.visitors`]): entities that visit traversed parts of the tree when it ticks
        pre_tick_handlers ([:obj:`func`]): functions that run before the entire tree is ticked
        post_tick_handlers ([:obj:`func`]): functions that run after the entire tree is ticked

    Raises:
        AssertionError: if incoming root variable is not the correct type
    """
    def __init__(self, root):
        self.count = 0
        assert root is not None, "root node must not be 'None'"
        assert isinstance(root, behaviour.Behaviour), "root node must be an instance of or descendant of pytrees.behaviour.Behaviour"
        self.root = root
        self.visitors = []
        self.pre_tick_handlers = []
        self.post_tick_handlers = []
        self.interrupt_tick_tocking = False
        self.tree_update_handler = None  # child classes can utilise this one

    def add_pre_tick_handler(self, handler):
        """
        Add a function to execute before the tree is ticked. The function must have
        a single argument of type :class:`~py_trees.trees.BehaviourTree`.

        Some ideas that are often used:

        * logging (to file or stdout)
        * modifications on the tree itself (e.g. starting a new plan)

        Args:
            handler (:obj:`func`): function
        """
        self.pre_tick_handlers.append(handler)

    def add_post_tick_handler(self, handler):
        """
        Add a function to execute after the tree has ticked. The function must have
        a single argument of type :class:`~py_trees.trees.BehaviourTree`.

        Some ideas that are often used:

        * logging
        * modifications on the tree itself (e.g. closing down a plan)
        * sending data to visualisation tools
        * introspect the state of the tree to make and send reports

        Args:
            handler (:obj:`func`): function
        """
        self.post_tick_handlers.append(handler)

    def prune_subtree(self, unique_id):
        """
        Prune a subtree given the unique id of the root of the subtree.

        Args:
            unique_id (uuid.UUID): unique id of the subtree root

        Returns:
            :obj:`bool`: success or failure of the operation

        Raises:
            AssertionError: if unique id is the behaviour tree's root node id
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
        Insert a subtree as a child of the specified parent. If the parent
        is found, this directly calls the parent's
        :meth:`~py_trees.composites.Composite.insert_child`
        method using the child and index arguments.

        Args:
            child (:class:`~py_trees.behaviour.Behaviour`): subtree to insert
            unique_id (uuid.UUID): unique id of the parent
            index (:obj:`int`): insert the child at this index, pushing all children after it back one.

        Returns:
            :obj:`bool`: suceess or failure (parent not found) of the operation

        Raises:
            AssertionError: if the parent is not a :class:`~py_trees.composites.Composite`

        .. todo::

           Could use better, more informative error handling here. Especially if the insertion
           has its own error handling (e.g. index out of range). Could also use a different api
           that relies on the id of the sibling node it should be inserted before/after.
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

        Args:
            unique_id (uuid.UUID): unique id of the parent
            subtree (:class:`~py_trees.behaviour.Behaviour`): root behaviour of the subtree

        Raises
            AssertionError: if unique id is the behaviour tree's root node id

        Returns:
            :obj:`bool`: suceess or failure of the operation
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
         Relays to calling the :meth:`~py_trees.behaviour.Behaviuor.setup` method
         on the root behaviour. This in turn should get recursively called down through
         the entire tree.

        Args:
             timeout (:obj:`float`): time to wait (0.0 is blocking forever)

        Return:
            :obj:`bool`: suceess or failure of the operation
        """
        return self.root.setup(timeout)

    def tick(self, pre_tick_handler=None, post_tick_handler=None):
        """
        Tick the tree just once and run any handlers before and after the tick.
        This optionally accepts some one-shot handlers (c.f. those added by
        :meth:`~py_trees.trees.BehaviourTree.add_pre_tick_handler` and :meth:`~py_trees.trees.BehaviourTree.add_post_tick_handler`
        which will be automatically run every time).

        The handler functions must have a single argument of type :class:`~py_trees.trees.BehaviourTree`.

        Args:
            pre_tick_handler (:obj:`func`): function to execute before ticking
            post_tick_handler (:obj:`func`): function to execute after ticking
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
        Tick continuously with a sleep interval as specified. This optionally accepts some handlers that will
        be used for the duration of this tick tock (c.f. those added by
        :meth:`~py_trees.trees.BehaviourTree.add_pre_tick_handler` and :meth:`~py_trees.trees.BehaviourTree.add_post_tick_handler`
        which will be automatically run every time).

        The handler functions must have a single argument of type :class:`~py_trees.trees.BehaviourTree`.

        Args:
            sleep_ms (:obj:`float`): sleep this much between ticks (milliseconds)
            number_of_iterations (:obj:`int`): number of iterations to tick-tock
            pre_tick_handler (:obj:`func`): function to execute before ticking
            post_tick_handler (:obj:`func`): function to execute after ticking
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

    def tip(self):
        """
        Get the *tip* of the tree.
        This corresponds to the the deepest node that was running before the
        subtree traversal reversed direction and headed back to this node.

        Returns:
            :class:`~py_trees.behaviour.Behaviour` or :obj:`None`: child behaviour, itself or :obj:`None` if its status is :data:`~py_trees.common.Status.INVALID`

        .. seealso:: :meth:`~py_trees.behaviour.Behaviour.tip`
        """
        return self.root.tip()

    def interrupt(self):
        """
        Interrupt tick-tock if it is tick-tocking. Note that this will permit a currently
        executing tick to finish before interrupting the tick-tock.
        """
        self.interrupt_tick_tocking = True

    def destroy(self):
        """
        Destroy the tree by stopping the root node.
        """
        self.root.stop()

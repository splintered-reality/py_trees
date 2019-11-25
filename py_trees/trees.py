#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/splintered-reality/py_trees/devel/LICENSE
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

import functools
import os
import signal
import threading
import time

from . import behaviour
from . import common
from . import composites
from . import display
from . import visitors

CONTINUOUS_TICK_TOCK = -1

##############################################################################
# Methods
##############################################################################


def setup(root: behaviour.Behaviour,
          timeout: float=common.Duration.INFINITE,
          visitor: visitors.VisitorBase=None,
          **kwargs: int):
    """
    Crawls across a (sub)tree of behaviours
    calling :meth:`~py_trees.behaviour.Behaviour.setup` on each behaviour.

    Visitors can optionally be provided to provide a node-by-node analysis
    on the result of each node's :meth:`~py_trees.behaviour.Behaviour.setup`
    before the next node's :meth:`~py_trees.behaviour.Behaviour.setup` is called.
    This is useful on trees with relatively long setup times to progressively
    report out on the current status of the operation.

    Args:
        root: unmanaged (sub)tree root behaviour
        timeout: time (s) to wait (use common.Duration.INFINITE to block indefinitely)
        visitor: runnable entities on each node after it's setup
        **kwargs: dictionary of arguments to distribute to all behaviours in the (sub) tree

    Raises:
        Exception: be ready to catch if any of the behaviours raise an exception
        RuntimeError: in case setup() times out
    """
    # SIGUSR1 is a better choice since it's a user defined operation, but these
    # are not available on windows, so overload one of the standard definitions
    try:
        _SIGNAL = signal.SIGUSR1
    except AttributeError:  # windows...
        # SIGINT can get you into trouble if for example, you are using a
        # process manager that plays shenanigans with SIGINT. Nonetheless,
        # it will work in most situations. If a windows user is running into
        # problems, work with them to resolve it.
        _SIGNAL = signal.SIGINT

    def on_timer_timed_out():
        os.kill(os.getpid(), _SIGNAL)

    def signal_handler(unused_signum, unused_frame, original_signal_handler):
        signal.signal(_SIGNAL, original_signal_handler)
        raise RuntimeError("tree setup interrupted or timed out")

    def visited_setup():
        if visitor is not None:
            visitor.initialise()
        for node in root.iterate():
            node.setup(**kwargs)
            if visitor is not None:
                node.visit(visitor)
        if visitor is not None:
            visitor.finalise()

    if timeout == common.Duration.INFINITE:
        visited_setup()
    else:
        original_signal_handler = signal.getsignal(_SIGNAL)
        signal.signal(
            _SIGNAL,
            functools.partial(
                signal_handler,
                original_signal_handler=original_signal_handler)
        )
        try:
            timer = threading.Timer(interval=timeout, function=on_timer_timed_out)
            timer.start()
            visited_setup()
        finally:
            timer.cancel()  # this only works if the timer is still waiting
            signal.signal(_SIGNAL, original_signal_handler)

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
        TypeError: if root variable is not an instance of :class:`~py_trees.behaviour.Behaviour`
    """
    def __init__(self, root: behaviour.Behaviour):
        self.count = 0
        if not isinstance(root, behaviour.Behaviour):
            raise TypeError("root node must be an instance of 'py_trees.behaviour.Behaviour' [{}]".format(type(root)))
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

    def add_visitor(self, visitor):
        """
        Trees can run multiple visitors on each behaviour as they
        tick through a tree.

        Args:
            visitor (:class:`~py_trees.visitors.VisitorBase`): sub-classed instance of a visitor

        .. seealso:: :class:`~py_trees.visitors.DebugVisitor`,
            :class:`~py_trees.visitors.SnapshotVisitor`,
            :class:`~py_trees.visitors.DisplaySnapshotVisitor`
        """
        self.visitors.append(visitor)

    def prune_subtree(self, unique_id):
        """
        Prune a subtree given the unique id of the root of the subtree.

        Args:
            unique_id (uuid.UUID): unique id of the subtree root

        Returns:
            :obj:`bool`: success or failure of the operation

        Raises:
            RuntimeError: if unique id is the behaviour tree's root node id
        """
        # TODO: convert this to throwing exceptions instead
        if self.root.id == unique_id:
            raise RuntimeError("may not prune the root node")
        for child in self.root.iterate():
            if child.id == unique_id:
                parent = child.parent
                if parent is not None:
                    parent.remove_child(child)
                    if self.tree_update_handler is not None:
                        self.tree_update_handler()
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
            TypeError: if the parent is not a :class:`~py_trees.composites.Composite`

        .. todo::

           Could use better, more informative error handling here. Especially if the insertion
           has its own error handling (e.g. index out of range). Could also use a different api
           that relies on the id of the sibling node it should be inserted before/after.
        """
        # TODO: convert this to throwing exceptions instead
        for node in self.root.iterate():
            if node.id == unique_id:
                if not isinstance(node, composites.Composite):
                    raise TypeError("parent must be a Composite behaviour.")
                node.insert_child(child, index)
                if self.tree_update_handler is not None:
                    self.tree_update_handler()
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
        # TODO: convert this to throwing exceptions instead
        if self.root.id == unique_id:
            raise RuntimeError("may not replace the root node")
        for child in self.root.iterate():
            if child.id == unique_id:
                parent = child.parent
                if parent is not None:
                    parent.replace_child(child, subtree)
                    if self.tree_update_handler is not None:
                        self.tree_update_handler()
                    return True
        return False

    def setup(self,
              timeout: float=common.Duration.INFINITE,
              visitor: visitors.VisitorBase=None,
              **kwargs):
        """
        Crawls across the tree calling :meth:`~py_trees.behaviour.Behaviour.setup`
        on each behaviour.

        Visitors can optionally be provided to provide a node-by-node analysis
        on the result of each node's :meth:`~py_trees.behaviour.Behaviour.setup`
        before the next node's :meth:`~py_trees.behaviour.Behaviour.setup` is called.
        This is useful on trees with relatively long setup times to progressively
        report out on the current status of the operation.

        Args:
            timeout (:obj:`float`): time (s) to wait (use common.Duration.INFINITE to block indefinitely)
            visitor (:class:`~py_trees.visitors.VisitorBase`): runnable entities on each node after it's setup
            **kwargs (:obj:`dict`): distribute arguments to this
               behaviour and in turn, all of it's children

        Raises:
            Exception: be ready to catch if any of the behaviours raise an exception
            RuntimeError: in case setup() times out
        """
        setup(
            root=self.root,
            timeout=timeout,
            visitor=visitor,
            **kwargs
        )

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
        if pre_tick_handler is not None:
            pre_tick_handler(self)
        for handler in self.pre_tick_handlers:
            handler(self)
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
        for visitor in self.visitors:
            visitor.finalise()
        for handler in self.post_tick_handlers:
            handler(self)
        if post_tick_handler is not None:
            post_tick_handler(self)
        self.count += 1

    def tick_tock(self,
                  period_ms,
                  number_of_iterations=CONTINUOUS_TICK_TOCK,
                  pre_tick_handler=None,
                  post_tick_handler=None):
        """
        Tick continuously with period as specified. Depending on the implementation, the
        period may be more or less accurate and may drift in some cases (the default
        implementation here merely assumes zero time in tick and sleeps for this duration
        of time and consequently, will drift).

        This optionally accepts some handlers that will
        be used for the duration of this tick tock (c.f. those added by
        :meth:`~py_trees.trees.BehaviourTree.add_pre_tick_handler` and :meth:`~py_trees.trees.BehaviourTree.add_post_tick_handler`
        which will be automatically run every time).

        The handler functions must have a single argument of type :class:`~py_trees.trees.BehaviourTree`.

        Args:
            period_ms (:obj:`float`): sleep this much between ticks (milliseconds)
            number_of_iterations (:obj:`int`): number of iterations to tick-tock
            pre_tick_handler (:obj:`func`): function to execute before ticking
            post_tick_handler (:obj:`func`): function to execute after ticking
        """
        tick_tocks = 0
        while not self.interrupt_tick_tocking and (tick_tocks < number_of_iterations or number_of_iterations == CONTINUOUS_TICK_TOCK):
            self.tick(pre_tick_handler, post_tick_handler)
            try:
                time.sleep(period_ms / 1000.0)
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

    def shutdown(self):
        """
        Crawls across the tree calling :meth:`~py_trees.behaviour.Behaviour.shutdown`
        on each behaviour.

        Raises:
            Exception: be ready to catch if any of the behaviours raise an exception
        """
        # TODO: this method is still quite naive .. could use similar visitors and
        # timeout mechanisms as used in setup()
        for node in self.root.iterate():
            node.shutdown()

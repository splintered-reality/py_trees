#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/splintered-reality/py_trees/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
Tree stewardship.

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

from __future__ import annotations

import functools
import os
import signal
import threading
import time
import types
import typing
import uuid

from . import behaviour, common, composites, visitors

CONTINUOUS_TICK_TOCK = -1

##############################################################################
# Methods
##############################################################################


def setup(
    root: behaviour.Behaviour,
    timeout: typing.Union[float, common.Duration] = common.Duration.INFINITE,
    visitor: typing.Optional[visitors.VisitorBase] = None,
    **kwargs: int,
) -> None:
    """
    Crawl across a (sub)tree of behaviours calling :meth:`~py_trees.behaviour.Behaviour.setup` on each behaviour.

    Visitors can optionally be provided to provide a node-by-node analysis
    on the result of each node's :meth:`~py_trees.behaviour.Behaviour.setup`
    before the next node's :meth:`~py_trees.behaviour.Behaviour.setup` is called.
    This is useful on trees with relatively long setup times to progressively
    report out on the current status of the operation.

    Args:
        root: unmanaged (sub)tree root behaviour
        timeout: time (s) to wait (use common.Duration.INFINITE to block indefinitely)
        visitor: runnable entities on each node after it's setup
        **kwargs: dictionary of args to distribute to all behaviours in the (sub)tree

    Raises:
        Exception: be ready to catch if any of the behaviours raise an exception
        RuntimeError: in case setup() times out
    """
    # SIGUSR1 is a better choice since it's a user defined operation, but these
    # are not available on windows, so overload one of the standard definitions
    try:
        _SIGNAL = signal.SIGUSR1  # noqa
    except AttributeError:  # windows...
        # SIGINT can get you into trouble if for example, you are using a
        # process manager that plays shenanigans with SIGINT. Nonetheless,
        # it will work in most situations. If a windows user is running into
        # problems, work with them to resolve it.
        _SIGNAL = signal.SIGINT  # noqa
    # This will be used as a global variable in the signal handler.
    # Mypy has trouble with global variables that are not module variables.
    #   https://github.com/python/mypy/issues/5732
    current_behaviour_name: typing.Optional[str] = None

    def on_timer_timed_out() -> None:
        os.kill(os.getpid(), _SIGNAL)

    def signal_handler(
        unused_signum: int,
        unused_frame: types.FrameType,
        original_signal_handler: typing.Optional[signal.Handlers],
    ) -> None:
        global current_behaviour_name
        signal.signal(_SIGNAL, original_signal_handler)
        raise RuntimeError(
            f"tree setup interrupted or timed out [{current_behaviour_name}]"  # type: ignore[name-defined]
        )

    def visited_setup() -> None:
        global current_behaviour_name
        if visitor is not None:
            visitor.initialise()
        for node in root.iterate():
            current_behaviour_name = node.name  # type: ignore[name-defined]
            node.setup(**kwargs)
            if visitor is not None:
                node.visit(visitor)
        if visitor is not None:
            visitor.finalise()

    if isinstance(timeout, common.Duration):
        timeout = timeout.value
    if timeout == common.Duration.INFINITE.value:  # math.inf
        visited_setup()
    else:
        original_signal_handler = signal.getsignal(_SIGNAL)
        signal.signal(
            _SIGNAL,
            functools.partial(
                signal_handler, original_signal_handler=original_signal_handler
            ),
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
    Grow, water, prune your behaviour tree with this, the tree custodian.

    It features a few enhancements that go above and beyond just ticking
    the root behaviour of a tree. These provide richer logging,
    introspection and dynamic management of the tree itself:

    * Pre and post tick handlers to execute code automatically before and after a tick
    * Visitor access to the parts of the tree that were traversed in a tick
    * Subtree pruning and insertion operations
    * Continuous tick-tock support

    .. seealso:: The :ref:`py-trees-demo-tree-stewardship-program` program demonstrates the above features.

    Args:
        root (:class:`~py_trees.behaviour.Behaviour`): root node of the tree

    Attributes:
        count: number of times the tree has been ticked.
        root: root node of the tree
        visitors: entities that visit traversed parts of the tree when it ticks
        pre_tick_handlers: functions that run before the entire tree is ticked
        post_tick_handlers: functions that run after the entire tree is ticked

    Raises:
        TypeError: if root variable is not an instance of :class:`~py_trees.behaviour.Behaviour`
    """

    def __init__(self, root: behaviour.Behaviour):
        self.count: int = 0
        if not isinstance(root, behaviour.Behaviour):
            raise TypeError(
                "root node must be an instance of 'py_trees.behaviour.Behaviour' [{}]".format(
                    type(root)
                )
            )
        self.root: behaviour.Behaviour = root
        self.visitors: typing.List[visitors.VisitorBase] = []
        self.pre_tick_handlers: typing.List[
            typing.Callable[["BehaviourTree"], None]
        ] = []
        self.post_tick_handlers: typing.List[
            typing.Callable[["BehaviourTree"], None]
        ] = []
        self.interrupt_tick_tocking = False
        self.tree_update_handler: typing.Optional[typing.Callable[[], None]] = None

    def add_pre_tick_handler(
        self, handler: typing.Callable[["BehaviourTree"], None]
    ) -> None:
        """
        Add a function to execute before the tree is ticked.

        The function must have
        a single argument of type :class:`~py_trees.trees.BehaviourTree`.

        Some ideas that are often used:

        * logging (to file or stdout)
        * modifications on the tree itself (e.g. starting a new plan)

        Args:
            handler: function
        """
        self.pre_tick_handlers.append(handler)

    def add_post_tick_handler(
        self, handler: typing.Callable[["BehaviourTree"], None]
    ) -> None:
        """
        Add a function to execute after the tree has ticked.

        The function must have
        a single argument of type :class:`~py_trees.trees.BehaviourTree`.

        Some ideas that are often used:

        * logging
        * modifications on the tree itself (e.g. closing down a plan)
        * sending data to visualisation tools
        * introspect the state of the tree to make and send reports

        Args:
            handler: function
        """
        self.post_tick_handlers.append(handler)

    def add_visitor(self, visitor: visitors.VisitorBase) -> None:
        """
        Welcome a visitor.

        Trees can run multiple visitors on each behaviour as they
        tick through a tree.

        Args:
            visitor: sub-classed instance of a visitor

        .. seealso:: :class:`~py_trees.visitors.DebugVisitor`,
            :class:`~py_trees.visitors.SnapshotVisitor`,
            :class:`~py_trees.visitors.DisplaySnapshotVisitor`
        """
        self.visitors.append(visitor)

    def prune_subtree(self, unique_id: uuid.UUID) -> bool:
        """
        Prune a subtree given the unique id of the root of the subtree.

        Args:
            unique id of the subtree root

        Returns:
            success or failure of the operation

        Raises:
            RuntimeError: if unique id is the root or parent does not have remove_node
        """
        # TODO: convert this to throwing exceptions instead
        if self.root.id == unique_id:
            raise RuntimeError("may not prune the root node")
        for child in self.root.iterate():
            if child.id == unique_id:
                parent = child.parent
                if parent is not None:
                    parent_remove_child = getattr(parent, "remove_child", None)
                    if callable(parent_remove_child):
                        parent_remove_child(child)
                    else:
                        raise RuntimeError(
                            f"parent type does not have 'remove_child' [{type(parent)}]"
                        )
                    if self.tree_update_handler is not None:
                        self.tree_update_handler()
                    return True
        return False

    def insert_subtree(
        self, child: behaviour.Behaviour, unique_id: uuid.UUID, index: int
    ) -> bool:
        """
        Insert a subtree as a child of the specified parent.

        If the parent is found, this directly calls the parent's
        :meth:`~py_trees.composites.Composite.insert_child`
        method using the child and index arguments.

        Args:
            child: subtree to insert
            unique_id: unique id of the parent
            index: insert the child at this index, pushing all children after it back one.

        Returns:
            suceess or failure (parent not found) of the operation

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

    def replace_subtree(
        self, unique_id: uuid.UUID, subtree: behaviour.Behaviour
    ) -> bool:
        """
        Replace the subtree with the specified id for the new subtree.

        This is a common pattern where we'd like to swap out a whole sub-behaviour for another one.

        Args:
            unique_id: unique id of the parent
            subtree: root behaviour of the subtree

        Raises
            AssertionError: if unique id is the behaviour tree's root node id

        Returns:
            : suceess or failure of the operation
        """
        # TODO: convert this to throwing exceptions instead
        if self.root.id == unique_id:
            raise RuntimeError("may not replace the root node")
        for child in self.root.iterate():
            if child.id == unique_id:
                parent = child.parent
                if parent is not None:
                    parent_replace_child = getattr(parent, "replace_child", None)
                    if callable(parent_replace_child):
                        parent_replace_child(child, subtree)
                    else:
                        raise RuntimeError(
                            f"parent type does not have 'replace_child' [{type(parent)}]"
                        )
                    #                    parent.replace_child(child, subtree)
                    if self.tree_update_handler is not None:
                        self.tree_update_handler()
                    return True
        return False

    def setup(
        self,
        timeout: typing.Union[float, common.Duration] = common.Duration.INFINITE,
        visitor: typing.Optional[visitors.VisitorBase] = None,
        **kwargs: int,
    ) -> None:
        """
        Crawl across the tree calling :meth:`~py_trees.behaviour.Behaviour.setup` on each behaviour.

        Visitors can optionally be provided to provide a node-by-node analysis
        on the result of each node's :meth:`~py_trees.behaviour.Behaviour.setup`
        before the next node's :meth:`~py_trees.behaviour.Behaviour.setup` is called.
        This is useful on trees with relatively long setup times to progressively
        report out on the current status of the operation.

        Args:
            timeout: time (s) to wait (use common.Duration.INFINITE to block indefinitely)
            visitor: runnable entities on each node after it's setup
            **kwargs: distribute args to this behaviour and in turn, to it's children

        Raises:
            Exception: be ready to catch if any of the behaviours raise an exception
            RuntimeError: in case setup() times out
        """
        setup(root=self.root, timeout=timeout, visitor=visitor, **kwargs)

    def tick(
        self: BehaviourTree,
        pre_tick_handler: typing.Optional[
            typing.Callable[[BehaviourTree], None]
        ] = None,
        post_tick_handler: typing.Optional[
            typing.Callable[[BehaviourTree], None]
        ] = None,
    ) -> None:
        """
        Tick the tree just once and run any handlers before and after the tick.

        This optionally accepts some one-shot handlers (c.f. those added by
        :meth:`~py_trees.trees.BehaviourTree.add_pre_tick_handler` and
        :meth:`~py_trees.trees.BehaviourTree.add_post_tick_handler`
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

    def tick_tock(
        self: BehaviourTree,
        period_ms: int,
        number_of_iterations: int = CONTINUOUS_TICK_TOCK,
        pre_tick_handler: typing.Optional[
            typing.Callable[[BehaviourTree], None]
        ] = None,
        post_tick_handler: typing.Optional[
            typing.Callable[[BehaviourTree], None]
        ] = None,
    ) -> None:
        """
        Tick continuously with period as specified.

        Depending on the implementation, the
        period may be more or less accurate and may drift in some cases (the default
        implementation here merely assumes zero time in tick and sleeps for this duration
        of time and consequently, will drift).

        This optionally accepts some handlers that will
        be used for the duration of this tick tock (c.f. those added by
        :meth:`~py_trees.trees.BehaviourTree.add_pre_tick_handler`
        and :meth:`~py_trees.trees.BehaviourTree.add_post_tick_handler`
        which will be automatically run every time).

        The handler functions must have a single argument of type :class:`~py_trees.trees.BehaviourTree`.

        Args:
            period_ms (:obj:`float`): sleep this much between ticks (milliseconds)
            number_of_iterations (:obj:`int`): number of iterations to tick-tock
            pre_tick_handler (:obj:`func`): function to execute before ticking
            post_tick_handler (:obj:`func`): function to execute after ticking
        """
        tick_tocks = 0
        while not self.interrupt_tick_tocking and (
            tick_tocks < number_of_iterations
            or number_of_iterations == CONTINUOUS_TICK_TOCK
        ):
            self.tick(pre_tick_handler, post_tick_handler)
            try:
                time.sleep(period_ms / 1000.0)
            except KeyboardInterrupt:
                break
            tick_tocks += 1
        self.interrupt_tick_tocking = False

    def tip(self) -> typing.Optional[behaviour.Behaviour]:
        """
        Get the *tip* of the tree.

        Returns:
            The deepest node (behaviour) that was running before subtree traversal
            reversed direction, or None if this behaviour's status is
            :data:`~py_trees.common.Status.INVALID`.

        .. seealso:: :meth:`~py_trees.behaviour.Behaviour.tip`
        """
        return self.root.tip()

    def interrupt(self) -> None:
        """
        Interrupt tick-tock if it is tick-tocking.

        Note that this will permit a currently
        executing tick to finish before interrupting the tick-tock.
        """
        self.interrupt_tick_tocking = True

    def shutdown(self) -> None:
        """
        Crawl across the tree, calling :meth:`~py_trees.behaviour.Behaviour.shutdown` on each behaviour.

        Raises:
            Exception: be ready to catch if any of the behaviours raise an exception
        """
        # TODO: this method is still quite naive .. could use similar visitors and
        # timeout mechanisms as used in setup()
        for node in self.root.iterate():
            node.shutdown()

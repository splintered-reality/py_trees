#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/yujinrobot/gopher_crazy_hospital/license/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
.. module:: trees
   :platform: Unix
   :synopsis: Managing a behaviour tree.

This module creates tools for managing your entire behaviour tree.

----

"""

##############################################################################
# Imports
##############################################################################

import rospy
import std_msgs.msg as std_msgs
import time

from . import common
from . import display
from .behaviours import Behaviour
from .composites import Composite


##############################################################################
# Classes
##############################################################################

CONTINUOUS_TICK_TOCK = -1

##############################################################################
# Classes
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
                assert isinstance(node, Composite), "parent must be a Composite behaviour."
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

    def tick(self, pre_tick_visitor, post_tick_visitor):
        """
        Tick over the tree just once.

        :param pre_tick_visitor: visitor that runs on this class instance before the tick
        :type pre_tick_visitor: any class with a run(py_trees.BehaviourTree) method
        :param post_tick_visitor: visitor that runs on this class instance before the tick
        :type post_tick_visitor: any class with a run(py_trees.BehaviourTree) method
        """
        # pre
        for visitor in self.pre_tick_handlers:
            visitor.run(self)
        if pre_tick_visitor is not None:
            pre_tick_visitor.run(self)
        for visitor in self.visitors:
            visitor.initialise()
        # tick
        for node in self.root.tick():
            for visitor in self.visitors:
                node.visit(visitor)
        # post
        for visitor in self.post_tick_handlers:
            visitor.run(self)
        if post_tick_visitor is not None:
            post_tick_visitor.run(self)
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
            # pre
            for visitor in self.pre_tick_handlers:
                visitor.run(self)
            if pre_tick_handler is not None:
                pre_tick_handler(self)
            for visitor in self.visitors:
                visitor.initialise()
            # tick
            for node in self.root.tick():
                for visitor in self.visitors:
                    node.visit(visitor)
            # post
            for handler in self.post_tick_handlers:
                handler(self)
            if post_tick_handler is not None:
                post_tick_handler(self)
            try:
                time.sleep(sleep_ms / 1000.0)
            except KeyboardInterrupt:
                break
            tick_tocks += 1
            self.count += 1
        self.interrupt_tick_tocking = False

    def interrupt(self):
        """
        Interrupt tick-tock if it is tick-tocking.
        """
        self.interrupt_tick_tocking = True

    def destroy(self):
        """
        Destroy the tree by aborting the root node
        """
        self.root.abort()


class ROSBehaviourTree(BehaviourTree):
    """
    Wrap the :py:class:`BehaviourTree <py_trees.trees.BehaviourTree>` class with
    a few latched publishers that publish representations of both the full tree and the
    runtime iteration on that tree.

    Publishers:

     - ~/ascii_tree (std_msgs/String) : ascii representation of the entire tree.
     - ~/snapshot/ascii_tree (std_msgs/String) : runtime snapshot of the ascii tree.
     - ~/dot_tree (std_msgs/String) : dot graph representation of the entire tree.
     - ~/snapshot/dot_tree (std_msgs/String) : runtime snapshot of the dot tree.
    """

    class SnapshotVisitor:
        """
        Visits the tree in tick-tock, recording runtime information for publishing
        the information as a snapshot view of the tree after the iteration has
        finished.

        Puts all iterated nodes into a dictionary, key: unique id value: status
        """
        def __init__(self):
            self.nodes = {}
            self.running_nodes = []
            self.previously_running_nodes = []

        def initialise(self):
            self.nodes = {}
            self.previously_running_nodes = self.running_nodes
            self.running_nodes = []

        def run(self, behaviour):
            self.nodes[behaviour.id] = behaviour.status
            if behaviour.status == common.Status.RUNNING:
                self.running_nodes.append(behaviour.id)

    def __init__(self, root):
        """
        Initialise the tree with a root.

        :param root: root node of the tree.
        :type root: instance or descendant of :py:class:`Behaviour <py_trees.behaviours.Behaviour>`
        :raises AssertionError: if incoming root variable is not the correct type.
        """
        super(ROSBehaviourTree, self).__init__(root)
        self.ascii_tree_publisher = rospy.Publisher('~ascii_tree', std_msgs.String, queue_size=1, latch=True)
        self.snapshot_ascii_tree_publisher = rospy.Publisher('~tick/ascii_tree', std_msgs.String, queue_size=1, latch=True)
        self.dot_tree_publisher = rospy.Publisher('~dot_tree', std_msgs.String, queue_size=1, latch=True)
        self.snapshot_dot_tree_publisher = rospy.Publisher('~tick/dot_tree', std_msgs.String, queue_size=1, latch=True)
        # tree_update_handler is in the base class, set this to the callback function here.
        self.tree_update_handler = self.publish_tree_modifications
        self.snapshot_visitor = ROSBehaviourTree.SnapshotVisitor()
        self.visitors.append(self.snapshot_visitor)
        self.post_tick_handlers.append(self.publish_tree_snapshots)

    def publish_tree_modifications(self, root):
        """
        Publishes updates when the whole tree has been modified.

        This function is passed in as a visitor to the underlying behaviour tree and triggered
        when there has been a change.
        """
        self.ascii_tree_publisher.publish(std_msgs.String(display.ascii_tree(self.root)))
        self.dot_tree_publisher.publish(std_msgs.String(display.stringify_dot_tree(self.root)))

    def publish_tree_snapshots(self, tree):
        """
        Callback that runs on a :py:class:`BehaviourTree <py_trees.trees.BehaviourTree>` after
        it has ticked.

        :param tree: the behaviour tree
        :type tree: :py:class:`BehaviourTree <py_trees.trees.BehaviourTree>`
        """
        snapshot = "\n\n%s" % display.ascii_tree(self.root, snapshot_information=self.snapshot_visitor)
        self.snapshot_ascii_tree_publisher.publish(std_msgs.String(snapshot))

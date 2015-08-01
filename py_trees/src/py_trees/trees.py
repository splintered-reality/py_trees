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

#import logging

from .behaviours import Behaviour

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
    :vartype visitors: list of classes that implement a run(py_trees.Behaviour) method.
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

    def prune_subtree(self, unique_id):
        """
        :param uuid.UUID unique_id: unique id of the subtree root
        :raises AssertionError: if unique id is the behaviour tree's root node id
        :return: success or failure of the pruning
        :rtype: bool
        """
        assert self.root.id != unique_id, "may not prune the root node"
        (parent, child) = self._find_parent_child_pair(self.root, unique_id)
        if parent is not None:
            parent.remove_child(child)
            return True
        else:
            return False

    def _find_parent_child_pair(self, subtree, unique_id):
        """
        Find the parent of the child with the specified unique_id
        :param subtree: the part of the behaviour tree to search over
        :type root: instance or descendant of :class:`Behaviour <py_trees.behaviours.Behaviour>`
        :param uuid.UUID unique_id: id of the child
        :return: parent-child pair
        :rtype: 2-tuple of instance or descendant of :class:`Behaviour <py_trees.behaviours.Behaviour>`
        """
        for child in subtree.children:
            if child.id == unique_id:
                return (subtree, child)
            (descendant_parent, descendant_child) = self._find_parent_child_pair(child, unique_id)
            if descendant_parent is not None:
                return (descendant_parent, descendant_child)
        return (None, None)

#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/splintered-reality/py_trees/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
Visitors are entities that can be passed to a tree implementation
(e.g. :class:`~py_trees.trees.BehaviourTree`) and used to either visit
each and every behaviour in the tree, or visit behaviours as the tree is
traversed in an executing tick. At each behaviour, the visitor
runs its own method on the behaviour to do as it wishes - logging, introspecting, etc.

.. warning:: Visitors should not modify the behaviours they visit.
"""

##############################################################################
# Imports
##############################################################################

##############################################################################
# Visitors
##############################################################################


class VisitorBase(object):
    """
    Parent template for visitor types.

    Visitors are primarily designed to work with :class:`~py_trees.trees.BehaviourTree`
    but they can be used in the same way for other tree custodian implementations.

    Args:
        full (:obj:`bool`): flag to indicate whether it should be used to visit only traversed nodes or the entire tree

    Attributes:
        full (:obj:`bool`): flag to indicate whether it should be used to visit only traversed nodes or the entire tree
    """
    def __init__(self, full=False):
        self.full = full

    def initialise(self):
        """
        Override this method if any resetting of variables needs to be
        performed between ticks (i.e. visitations).
        """
        pass

    def finalise(self):
        """
        Override this method if any work needs to be
        performed after ticks (i.e. showing data).
        """
        pass

    def run(self, behaviour):
        """
        This method gets run as each behaviour is ticked. Override it to
        perform some activity - e.g. introspect the behaviour
        to store/process logging data for visualisations.

        Args:
            behaviour (:class:`~py_trees.behaviour.Behaviour`): behaviour that is ticking
        """
        pass


class DebugVisitor(VisitorBase):
    """
    Picks up and logs feedback messages and the behaviour's status. Logging is done with
    the behaviour's logger.
    """
    def __init__(self):
        super(DebugVisitor, self).__init__(full=False)

    def run(self, behaviour):
        if behaviour.feedback_message:
            behaviour.logger.debug("%s.run() [%s][%s]" % (self.__class__.__name__, behaviour.feedback_message, behaviour.status))
        else:
            behaviour.logger.debug("%s.run() [%s]" % (self.__class__.__name__, behaviour.status))


class SnapshotVisitor(VisitorBase):
    """
    Visits the tree in tick-tock, recording the id/status of the visited set
    of nodes. Additionally caches the last tick's visited collection for
    comparison.

    Args:
        full (:obj:`bool`): flag to indicate whether it should be used to visit only traversed nodes or the entire tree

    Attributes:
        visited (dict): dictionary of behaviour id (uuid.UUID) and status (:class:`~py_trees.common.Status`) pairs
        previously_visited (dict): dictionary of behaviour id's saved from the previous tree tick

    .. seealso::

        This visitor is used with the :class:`~py_trees.trees.BehaviourTree` class to collect
        information and :meth:`py_trees.display.unicode_tree` to display information.
    """
    def __init__(self, full=False):
        super(SnapshotVisitor, self).__init__(full=full)
        self.visited = {}
        self.previously_visited = {}

    def initialise(self):
        """
        Cache the last collection of visited nodes and reset the dictionary.
        """
        self.previously_visited = self.visited
        self.visited = {}

    def run(self, behaviour):
        """
        This method gets run as each behaviour is ticked. Catch the id and status and store it.

        Args:
            behaviour (:class:`~py_trees.behaviour.Behaviour`): behaviour that is ticking
        """
        self.visited[behaviour.id] = behaviour.status


class WindsOfChangeVisitor(VisitorBase):
    """
    Visits the ticked part of a tree, checking off the status against the set of status
    results recorded in the previous tick. If there has been a change, it flags it.
    This is useful for determining when to trigger, e.g. logging.

    Attributes:
        changed (Bool): flagged if there is a difference in the visited path or :class:`~py_trees.common.Status` of any behaviour on the path
        ticked_nodes (dict): dictionary of behaviour id (uuid.UUID) and status (:class:`~py_trees.common.Status`) pairs from the current tick
        previously_ticked+nodes (dict): dictionary of behaviour id (uuid.UUID) and status (:class:`~py_trees.common.Status`) pairs from the previous tick
        running_nodes([uuid.UUID]): list of id's for behaviours which were traversed in the current tick
        previously_running_nodes([uuid.UUID]): list of id's for behaviours which were traversed in the last tick

    .. seealso:: The :ref:`py-trees-demo-logging-program` program demonstrates use of this visitor to trigger logging of a tree serialisation.
    """
    def __init__(self):
        super().__init__(full=False)
        self.changed = False
        self.ticked_nodes = {}
        self.previously_ticked_nodes = {}

    def initialise(self):
        """
        Switch running to previously running and then reset all other variables. This should
        get called before a tree ticks.
        """
        self.changed = False
        self.previously_ticked_nodes = self.ticked_nodes
        self.ticked_nodes = {}

    def run(self, behaviour):
        """
        This method gets run as each behaviour is ticked. Catch the id and status and store it.
        Additionally add it to the running list if it is :data:`~py_trees.common.Status.RUNNING`.

        Args:
            behaviour (:class:`~py_trees.behaviour.Behaviour`): behaviour that is ticking
        """
        self.ticked_nodes[behaviour.id] = behaviour.status
        try:
            if self.ticked_nodes[behaviour.id] != self.previously_ticked_nodes[behaviour.id]:
                self.changed = True
        except KeyError:
            self.changed = True

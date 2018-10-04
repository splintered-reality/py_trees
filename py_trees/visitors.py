#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/stonier/py_trees/devel/LICENSE
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

from . import common
# from . import console
# from . import syntax_highlighting

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
    Visits the tree in tick-tock, recording runtime information for publishing
    the information as a snapshot view of the tree after the iteration has
    finished.

    Args:
        full (:obj:`bool`): flag to indicate whether it should be used to visit only traversed nodes or the entire tree

    Attributes:
        nodes (dict): dictionary of behaviour id (uuid.UUID) and status (:class:`~py_trees.common.Status`) pairs
        running_nodes([uuid.UUID]): list of id's for behaviours which were traversed in the current tick
        previously_running_nodes([uuid.UUID]): list of id's for behaviours which were traversed in the last tick

    .. seealso::

        This visitor is used with the :class:`~py_trees.trees.BehaviourTree` class to collect
        information and :func:`~py_trees.display.ascii_tree` to display information.
    """
    def __init__(self, full=False):
        super(SnapshotVisitor, self).__init__(full=full)
        self.nodes = {}
        self.running_nodes = []
        self.previously_running_nodes = []

    def initialise(self):
        """
        Switch running to previously running and then reset all other variables. This will
        get called before a tree ticks.
        """
        self.nodes = {}
        self.previously_running_nodes = self.running_nodes
        self.running_nodes = []

    def run(self, behaviour):
        """
        This method gets run as each behaviour is ticked. Catch the id and status and store it.
        Additionally add it to the running list if it is :data:`~py_trees.common.Status.RUNNING`.

        Args:
            behaviour (:class:`~py_trees.behaviour.Behaviour`): behaviour that is ticking
        """
        self.nodes[behaviour.id] = behaviour.status
        if behaviour.status == common.Status.RUNNING:
            self.running_nodes.append(behaviour.id)

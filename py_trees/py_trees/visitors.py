#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/stonier/py_trees_suite/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
Various implementations of entities that visit
behaviour trees as they traverse through an executing tick.
Visitors must all possess two member methods:

* :meth:`initialise()`
* :meth:`run(behaviour)`

where behaviour is an instance of :class:`~py_trees.behaviour.Behaviour`
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


class DebugVisitor(object):
    """
    Picks up and logs status and feedback message (if set).
    """
    def __init__(self):
        super(DebugVisitor, self).__init__(full=False)

    def initialise(self):
        pass

    def run(self, behaviour):
        if behaviour.feedback_message:
            behaviour.logger.debug("%s.run() [%s][%s]" % (self.__class__.__name__, behaviour.feedback_message, behaviour.status))
        else:
            behaviour.logger.debug("%s.run() [%s]" % (self.__class__.__name__, behaviour.status))


class SnapshotVisitor(object):
    """
    Visits the tree in tick-tock, recording runtime information for publishing
    the information as a snapshot view of the tree after the iteration has
    finished.

    Attributes:
        nodes (dict): dictionary of behaviour id (uuid.UUID) and status (:class:`~py_trees.common.Status`) pairs
        running_nodes([uuid.UUID]): list of id's for behaviours which were traversed in the current tick
        previously_running_nodes([uuid.UUID]): list of id's for behaviours which were traversed in the last tick

    .. seealso::

        This visitor should be used with the :class:`~py_trees.trees.BehaviourTree` class to collect
        information and :func:`~py_trees.display.ascii_tree` to display information.
    """
    def __init__(self):
        super(SnapshotVisitor, self).__init__()
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

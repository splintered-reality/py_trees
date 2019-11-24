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

from . import blackboard
from . import console
from . import display

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
    Visits the ticked part of a tree, checking off the status against the set of status
    results recorded in the previous tick. If there has been a change, it flags it.
    This is useful for determining when to trigger, e.g. logging.

    Attributes:
        changed (Bool): flagged if there is a difference in the visited path or :class:`~py_trees.common.Status` of any behaviour on the path
        visited (dict): dictionary of behaviour id (uuid.UUID) and status (:class:`~py_trees.common.Status`) pairs from the current tick
        previously_visited (dict): dictionary of behaviour id (uuid.UUID) and status (:class:`~py_trees.common.Status`) pairs from the previous tick
        running_nodes([uuid.UUID]): list of id's for behaviours which were traversed in the current tick
        previously_running_nodes([uuid.UUID]): list of id's for behaviours which were traversed in the last tick
        visited_blackboard_ids(typing.Set[uuid.UUID]): blackboard client id's on the visited path
        visited_blackboard_keys(typing.Set[str]): blackboard variable keys on the visited path

    .. seealso:: The :ref:`py-trees-demo-logging-program` program demonstrates use of this visitor to trigger logging of a tree serialisation.
    """
    def __init__(self):
        super().__init__(full=False)
        self.changed = False
        self.visited = {}
        self.previously_visited = {}
        self.visited_blackboard_keys = set()
        self.visited_blackboard_client_ids = set()

    def initialise(self):
        """
        Switch running to previously running and then reset all other variables. This should
        get called before a tree ticks.
        """
        self.changed = False
        self.previously_visited = self.visited
        self.visited = {}
        self.visited_blackboard_keys = set()
        self.visited_blackboard_client_ids = set()

    def run(self, behaviour):
        """
        This method gets run as each behaviour is ticked. Catch the id and status and store it.
        Additionally add it to the running list if it is :data:`~py_trees.common.Status.RUNNING`.

        Args:
            behaviour (:class:`~py_trees.behaviour.Behaviour`): behaviour that is ticking
        """
        # behaviour status
        self.visited[behaviour.id] = behaviour.status
        try:
            if self.visited[behaviour.id] != self.previously_visited[behaviour.id]:
                self.changed = True
        except KeyError:
            self.changed = True
        # blackboards
        for blackboard in behaviour.blackboards:
            self.visited_blackboard_client_ids.add(blackboard.id())
            self.visited_blackboard_keys = self.visited_blackboard_keys | blackboard.read | blackboard.write | blackboard.exclusive


class DisplaySnapshotVisitor(SnapshotVisitor):
    """
    Visit the tree, capturing the visited path, it's changes since the last
    tick and additionally print the snapshot to console.

    Args:
        display_blackboard: print to the console the relevant part of the blackboard associated with behaviours on the visited path
        display_activity_stream: print to the console a log of the activity on the blackboard over the last tick
    """
    def __init__(
            self,
            display_blackboard: bool=False,
            display_activity_stream: bool=False
    ):
        super().__init__()
        self.display_blackboard = display_blackboard
        self.display_activity_stream = display_activity_stream
        if self.display_activity_stream:
            blackboard.Blackboard.enable_activity_stream()

    def initialise(self):
        self.root = None
        super().initialise()
        if self.display_activity_stream:
            blackboard.Blackboard.activity_stream.clear()

    def run(self, behaviour):
        self.root = behaviour  # last behaviour visited will always be the root
        super().run(behaviour)

    def finalise(self):
        print(
            "\n" +
            display.unicode_tree(
                root=self.root,
                show_status=False,
                visited=self.visited,
                previously_visited=self.previously_visited
            )
        )
        if self.display_blackboard:
            print(display.unicode_blackboard(key_filter=self.visited_blackboard_keys))
        if self.display_activity_stream:
            print(display.unicode_blackboard_activity_stream())

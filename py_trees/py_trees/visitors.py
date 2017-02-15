#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/stonier/py_trees_suite/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
.. module:: visitors
   :synopsis: Visitor parent interface and various implementations

Oh my spaghettified magnificence,
Bless my noggin with a tickle from your noodly appendages!
"""

##############################################################################
# Imports
##############################################################################

from . import common
# from . import console
from . import logging
# from . import syntax_highlighting

##############################################################################
# Visitors
##############################################################################


class VisitorBase(object):
    """Base object for visitors.
    """
    def __init__(self, full=False):
        """Initialises the base object for the visitor.

        :param bool full: if true, this visitor will visit all nodes in the tree
            every tick. If false, it only visits nodes that are parents of nodes
            that are running, as well as those running nodes.

        """
        self.full = full


class DebugVisitor(VisitorBase):
    """
    Picks up and logs status and feedback message (if set).
    """
    def __init__(self):
        super(DebugVisitor, self).__init__(full=False)
        self.logger = logging.Logger("Visitor")

    def initialise(self):
        pass

    def run(self, behaviour):
        if behaviour.feedback_message:
            self.logger.debug("  %s [%s][%s]" % (behaviour.name, behaviour.feedback_message, behaviour.status))
        else:
            self.logger.debug("  %s [%s]" % (behaviour.name, behaviour.status))


# class StatusVisitor(VisitorBase):
#     """
#     Display status, feedback message of visited behaviours in luvly syntax highlighted colours.
#     """
#     def __init__(self):
#         super(StatusVisitor, self).__init__(full=False)
#         self.name = "Visitor"
#         self.prefix = '{:<12}'.format(self.name) + " : "
#
#     def initialise(self):
#         pass
#
#     def run(self, behaviour):
#         if behaviour.feedback_message:
#             print(syntax_highlighting.status_colour_code(behaviour.status) + "[VISIT] " + self.prefix + "  %s [%s][%s]" % (behaviour.name, behaviour.feedback_message, str(behaviour.status)) + console.reset)
#         else:
#             print(syntax_highlighting.status_colour_code(behaviour.status) + "[VISIT] " + self.prefix + "  %s [%s]" % (behaviour.name, str(behaviour.status)) + console.reset)


class SnapshotVisitor(VisitorBase):
    """
    Visits the tree in tick-tock, recording runtime information for publishing
    the information as a snapshot view of the tree after the iteration has
    finished. This can be used for example, with the :py:function::`~py_trees.display.ascii_tree`
    function.

    Puts all iterated nodes into a dictionary, key: unique id value: status
    """
    def __init__(self):
        super(SnapshotVisitor, self).__init__()
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

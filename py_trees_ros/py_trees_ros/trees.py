#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/stonier/py_trees_suite/license/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
.. module:: trees
   :platform: Unix
   :synopsis: A tree caretaker with ros interfaces

Oh my spaghettified magnificence,
Bless my noggin with a tickle from your noodly appendages!
"""

##############################################################################
# Imports
##############################################################################

import datetime
import os
import py_trees.display
import py_trees.blackboard
import py_trees.common
import py_trees.conversions
import py_trees.trees
import py_trees_msgs.msg as py_trees_msgs
import rocon_python_comms
import rosbag
import rospkg
import rospy
import std_msgs.msg as std_msgs
import threading
import unique_id

from . import blackboard

##############################################################################
# ROS Trees
##############################################################################


class BehaviourTree(py_trees.trees.BehaviourTree):
    """
    Wrap the :py:class:`BehaviourTree <py_trees.trees.BehaviourTree>` class with
    a few latched publishers that publish representations of both the full tree and the
    runtime iteration on that tree.

    Publishers:

     - ~/ascii/tree (std_msgs/String) : static view of the entire tree.
     - ~/ascii/snapshot (std_msgs/String) : runtime snapshot of the ticking ascii tree.
     - ~/dot/tree (std_msgs/String) : static dot graph view of the entire tree.
     - ~/log/tree (std_msgs/String) : runtime view, with logging information of the entire tree for rqt/bagging
    """

    class SnapshotVisitor(py_trees.trees.VisitorBase):
        """
        Visits the tree in tick-tock, recording runtime information for publishing
        the information as a snapshot view of the tree after the iteration has
        finished.

        Puts all iterated nodes into a dictionary, key: unique id value: status
        """
        def __init__(self):
            super(BehaviourTree.SnapshotVisitor, self).__init__()
            self.nodes = {}
            self.running_nodes = []
            self.previously_running_nodes = []

        def initialise(self):
            self.nodes = {}
            self.previously_running_nodes = self.running_nodes
            self.running_nodes = []

        def run(self, behaviour):
            self.nodes[behaviour.id] = behaviour.status
            if behaviour.status == py_trees.common.Status.RUNNING:
                self.running_nodes.append(behaviour.id)

    class LoggingVisitor(py_trees.trees.VisitorBase):
        """
        Visits the entire tree and gathers logging information.
        """
        def __init__(self):
            super(BehaviourTree.LoggingVisitor, self).__init__()
            self.full = True  # examine all nodes

        def initialise(self):
            self.tree = py_trees_msgs.BehaviourTree()
            self.tree.header.stamp = rospy.Time.now()

        def run(self, behaviour):
            self.tree.behaviours.append(py_trees.conversions.behaviour_to_msg(behaviour))

    def __init__(self, root):
        """
        Initialise the tree with a root.

        :param root: root node of the tree.
        :type root: instance or descendant of :py:class:`Behaviour <py_trees.behaviours.Behaviour>`
        :raises AssertionError: if incoming root variable is not the correct type.
        """
        super(BehaviourTree, self).__init__(root)
        self.snapshot_visitor = BehaviourTree.SnapshotVisitor()
        self.logging_visitor = BehaviourTree.LoggingVisitor()
        self.visitors.append(self.snapshot_visitor)
        self.visitors.append(self.logging_visitor)
        self._bag_closed = False

        self.ros_blackboard = blackboard.Blackboard()

        now = datetime.datetime.now()
        topdir = rospkg.get_ros_home() + '/behaviour_trees'
        subdir = topdir + '/' + now.strftime('%Y-%m-%d')
        if not os.path.exists(topdir):
            os.makedirs(topdir)

        if not os.path.exists(subdir):
            os.makedirs(subdir)

        # opens in ros home directory for the user
        self.bag = rosbag.Bag(subdir + '/behaviour_tree_' + now.strftime("%H-%M-%S") + '.bag', 'w')

        self.last_tree = py_trees_msgs.BehaviourTree()
        self.lock = threading.Lock()

        # delay the publishers so we can instantiate this class without connecting to ros (private names need init_node)
        self.publishers = None

        # cleanup must come last as it assumes the existence of the bag
        rospy.on_shutdown(self.cleanup)

    def setup(self, timeout):
        """
        This gives control over the ros initialisation to the user...do all your ros stuff here!

        :param double timeout: time to wait (0.0 is blocking forever)
        :returns: whether it timed out trying to setup
        :rtype: boolean
        """
        self.setup_publishers()
        if not self.ros_blackboard.setup(timeout):
            return False
        self.post_tick_handlers.append(self.publish_tree_snapshots)
        self.post_tick_handlers.append(self.ros_blackboard.publish_blackboard)
        return super(BehaviourTree, self).setup(timeout)

    def setup_publishers(self):
        latched = True
        self.publishers = rocon_python_comms.utils.Publishers(
            [
                ("ascii_tree", "~ascii/tree", std_msgs.String, latched, 2),
                ("ascii_snapshot", "~ascii/snapshot", std_msgs.String, latched, 2),
                ("dot_tree", "~dot/tree", std_msgs.String, latched, 2),
                ("log_tree", "~log/tree", py_trees_msgs.BehaviourTree, latched, 2),
                ("tip", "~tip", py_trees_msgs.Behaviour, latched, 2)
            ]
        )

        # publish current state
        self.publish_tree_modifications(self.root)
        # set a handler to publish future modifiactions
        # tree_update_handler is in the base class, set this to the callback function here.
        self.tree_update_handler = self.publish_tree_modifications

    def publish_tree_modifications(self, tree):
        """
        Publishes updates when the whole tree has been modified.

        This function is passed in as a visitor to the underlying behaviour tree and triggered
        when there has been a change.
        """
        if self.publishers is None:
            rospy.logerr("BehaviourTree: call setup() on this tree to initialise the ros components")
            return
        self.publishers.ascii_tree.publish(std_msgs.String(py_trees.display.ascii_tree(self.root)))
        self.publishers.dot_tree.publish(std_msgs.String(py_trees.display.stringify_dot_tree(self.root)))

    def publish_tree_snapshots(self, tree):
        """
        Callback that runs on a :py:class:`BehaviourTree <py_trees.trees.BehaviourTree>` after
        it has ticked.

        :param tree: the behaviour tree
        :type tree: :py:class:`BehaviourTree <py_trees.trees.BehaviourTree>`
        """
        if self.publishers is None:
            rospy.logerr("BehaviourTree: call setup() on this tree to initialise the ros components")
            return
        snapshot = "\n\n%s" % py_trees.display.ascii_tree(self.root, snapshot_information=self.snapshot_visitor)
        self.publishers.ascii_snapshot.publish(std_msgs.String(snapshot))

        for behaviour in self.logging_visitor.tree.behaviours:
            behaviour.is_active = True if unique_id.fromMsg(behaviour.own_id) in self.snapshot_visitor.nodes else False
        # We're not interested in sending every single tree - only send a
        # message when the tree changes.
        if self.logging_visitor.tree.behaviours != self.last_tree.behaviours:
            if self.root.tip() is None:
                rospy.logerr("Behaviours: your tree is returning in an INVALID state (should always be FAILURE, RUNNING or SUCCESS)")
                return
            self.publishers.tip.publish(py_trees.conversions.behaviour_to_msg(self.root.tip()))
            self.publishers.log_tree.publish(self.logging_visitor.tree)
            with self.lock:
                if not self._bag_closed:
                    self.bag.write(self.publishers.log_tree.name, self.logging_visitor.tree)
            self.last_tree = self.logging_visitor.tree

    def cleanup(self):
        with self.lock:
            self.bag.close()
            self.interrupt_tick_tocking = True
            self._bag_closed = True

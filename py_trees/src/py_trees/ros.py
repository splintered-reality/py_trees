#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/stonier/py_trees_suite/license/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
.. module:: ros
   :platform: Unix
   :synopsis: Ros specific extensions

Oh my spaghettified magnificence,
Bless my noggin with a tickle from your noodly appendages!
"""

##############################################################################
# Imports
##############################################################################

import datetime
import operator
import os
import py_trees_msgs.msg as py_trees_msgs
import py_trees_msgs.srv as py_trees_srvs
import rocon_python_comms
import rosbag
import rospkg
import rospy
import rocon_console.console as console
import std_msgs.msg as std_msgs
import threading
import unique_id

from cPickle import dumps

from . import display
from . import blackboard
from . import common
from . import conversions
from . import trees

##############################################################################
# ROS Blackboard
##############################################################################


class Blackboard(object):
    """
    Manages :py:class:`Blackboard <py_trees.blackboard.Blackboard>`.
    Provides methods to initialize SubBlackboards to watch subset of Blackboard variables
    And publishers for publishing when variables in Blackboards or SubBlackboards are changed
    """
    class SubBlackboard(object):
        """
        Container class for a Subblackboard
        Keeps track of variables from Blackboard
        """
        def __init__(self, topic_name, attrs):
            self.blackboard = blackboard.Blackboard()
            self.topic_name = topic_name
            self.attrs = attrs
            self.dict = {}
            self.cached_dict = {}
            self.publisher = rospy.Publisher("~" + topic_name, std_msgs.String, latch=True, queue_size=2)

        def update_sub_blackboard(self):
            for attr in self.attrs:
                if '/' in attr:
                    check_attr = operator.attrgetter(".".join(attr.split('/')))
                else:
                    check_attr = operator.attrgetter(attr)
                try:
                    value = check_attr(self.blackboard)
                    self.dict[attr] = value
                except AttributeError:
                    pass

        def is_changed(self):
            self.update_sub_blackboard()
            current_pickle = dumps(self.dict, -1)
            blackboard_changed = current_pickle != self.cached_dict
            self.cached_dict = current_pickle

            return blackboard_changed

        def __str__(self):
            s = ""
            max_length = 0
            for k in self.dict.keys():
                max_length = len(k) if len(k) > max_length else max_length
            keys = sorted(self.dict)
            for key in keys:
                value = self.dict[key]
                if value is None:
                    value_string = "-"
                    s += console.cyan + "  " + '{0: <{1}}'.format(key, max_length + 1) + console.reset + ": " + console.yellow + "%s" % (value_string) + console.reset + "\n"
                else:
                    lines = ("%s" % value).split('\n')
                    if len(lines) > 1:
                        s += console.cyan + "  " + '{0: <{1}}'.format(key, max_length + 1) + console.reset + ":\n"
                        for line in lines:
                            s += console.yellow + "    %s" % line + console.reset + "\n"
                    else:
                        s += console.cyan + "  " + '{0: <{1}}'.format(key, max_length + 1) + console.reset + ": " + console.yellow + "%s" % (value) + console.reset + "\n"
            return s.rstrip()  # get rid of the trailing newline...print will take care of adding a new line

    def __init__(self):
        self.blackboard = blackboard.Blackboard()
        self.cached_blackboard_dict = {}
        self.sub_blackboards = []
        self.publisher = None

    def setup(self, timeout):
        """
        Typical py_trees setup function so the user can control when they want to do the
        ros setup. Do all of the ros initialisation in here.

        :param double timeout: time to wait (0.0 is blocking forever)
        :returns: whether it timed out trying to setup
        :rtype: boolean
        """
        self.publisher = rospy.Publisher("~blackboard", std_msgs.String, latch=True, queue_size=2)
        rospy.Service('~list_blackboard_variables', py_trees_srvs.BlackboardVariables, self.list_blackboard_variables_service)
        rospy.Service('~spawn_blackboard_watcher', py_trees_srvs.SpawnBlackboardWatcher, self.spawn_blackboard_watcher_service)
        rospy.Service('~destroy_blackboard_watcher', py_trees_srvs.DestroyBlackboardWatcher, self.destroy_blackboard_watcher_service)
        return True

    def get_nested_keys(self):
        variables = []

        def inner(v, k):
            for attr in dir(type(v)):
                if not isinstance(v, (bool, list, str, int, float)):
                        if not attr.startswith("_"):
                            value = getattr(v, attr)
                            if not callable(value):
                                if not attr.isupper():
                                    variables.append(k + "/" + attr)
                                    inner(value, k + "/" + attr)

        for key in sorted(self.blackboard.__dict__):
            variables.append(key)
            inner(self.blackboard.__dict__[key], key)

        return variables

    def initialize_sub_blackboard(self, attrs, topic_name=None):
        if isinstance(attrs, list):
            if not topic_name:
                topic_name = "sub_blackboard_" + str(len(self.sub_blackboards))

            sub_blackboard = Blackboard.SubBlackboard(topic_name, attrs)
            self.sub_blackboards.append(sub_blackboard)

        return topic_name

    def shutdown_sub_blackboard(self, req):
        for (i, sub_blackboard) in enumerate(self.sub_blackboards):
            if sub_blackboard.topic_name == req.topic_name:
                sub_blackboard.publisher.unregister()
                del self.sub_blackboards[i]
                return True
        return False

    def is_changed(self):
        current_pickle = dumps(self.blackboard.__dict__, -1)
        blackboard_changed = current_pickle != self.cached_blackboard_dict
        self.cached_blackboard_dict = current_pickle
        return blackboard_changed

    def publish_blackboard(self, tree):
        """
        Publishes the blackboard. Should be called at the end of every tick.
        """
        if self.publisher is None:
            rospy.logerr("Blacboard: call setup() on blackboard to initialise the ros components")
            return

        # publish blackboard
        if self.publisher.get_num_connections() > 0:
            if self.is_changed():
                self.publisher.publish("%s" % self.blackboard)

        # publish sub_blackboards
        if len(self.sub_blackboards) > 0:
            for (unused_i, sub_blackboard) in enumerate(self.sub_blackboards):
                if sub_blackboard.publisher.get_num_connections() > 0:
                    if sub_blackboard.is_changed():
                        sub_blackboard.publisher.publish("%s" % sub_blackboard)

    def destroy_blackboard_watcher_service(self, req):
        result = self.shutdown_sub_blackboard(req)
        return result

    def list_blackboard_variables_service(self, req):
        nested_keys = self.get_nested_keys()
        return py_trees_srvs.BlackboardVariablesResponse(nested_keys)

    def spawn_blackboard_watcher_service(self, req):
        topic_name = self.initialize_sub_blackboard(req.variables)
        if topic_name:
            absolute_topic_name = rospy.get_name() + "/" + topic_name
        else:
            absolute_topic_name = None
        return py_trees_srvs.SpawnBlackboardWatcherResponse(absolute_topic_name)


##############################################################################
# ROS Blackboard
##############################################################################

class BehaviourTree(trees.BehaviourTree):
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

    class SnapshotVisitor(trees.VisitorBase):
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
            if behaviour.status == common.Status.RUNNING:
                self.running_nodes.append(behaviour.id)

    class LoggingVisitor(trees.VisitorBase):
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
            self.tree.behaviours.append(conversions.behaviour_to_msg(behaviour))

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

        self.ros_blackboard = Blackboard()

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
        self.publishers.ascii_tree.publish(std_msgs.String(display.ascii_tree(self.root)))
        self.publishers.dot_tree.publish(std_msgs.String(display.stringify_dot_tree(self.root)))

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
        snapshot = "\n\n%s" % display.ascii_tree(self.root, snapshot_information=self.snapshot_visitor)
        self.publishers.ascii_snapshot.publish(std_msgs.String(snapshot))

        for behaviour in self.logging_visitor.tree.behaviours:
            behaviour.is_active = True if unique_id.fromMsg(behaviour.own_id) in self.snapshot_visitor.nodes else False
        # We're not interested in sending every single tree - only send a
        # message when the tree changes.
        if self.logging_visitor.tree.behaviours != self.last_tree.behaviours:
            if self.root.tip() is None:
                rospy.logerr("Behaviours: your tree is returning in an INVALID state (should always be FAILURE, RUNNING or SUCCESS)")
                return
            self.publishers.tip.publish(conversions.behaviour_to_msg(self.root.tip()))
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

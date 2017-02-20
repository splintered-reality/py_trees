#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/stonier/py_trees_suite/license/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
.. module:: blackboard
   :platform: Unix
   :synopsis: Ros wrapped blackboard

Oh my spaghettified magnificence,
Bless my noggin with a tickle from your noodly appendages!
"""

##############################################################################
# Imports
##############################################################################

import operator
import py_trees.blackboard
import py_trees_msgs.srv as py_trees_srvs
import rospy
import rocon_console.console as console
import std_msgs.msg as std_msgs

from cPickle import dumps

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
            self.blackboard = py_trees.blackboard.Blackboard()
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
        self.blackboard = py_trees.blackboard.Blackboard()
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

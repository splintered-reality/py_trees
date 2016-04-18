#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/yujinrobot/gopher_crazy_hospital/license/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
.. module:: dotcode_behaviour
   :platform: Unix
   :synopsis: Dot code a behaviour tree

Dotcode generation for a py_trees behaviour tree.
"""

##############################################################################
# Imports
##############################################################################

from __future__ import with_statement, print_function

import rospy
import py_trees_msgs.msg as py_trees_msgs
import unique_id

##############################################################################
# Classes
##############################################################################


class RosBehaviourTreeDotcodeGenerator(object):

    def __init__(self):
        self.last_drawargs = None
        self.dotcode = None
        self.firstcall = True
        self.rank = None
        self.rankdir = None
        self.ranksep = None
        self.graph = None
        self.dotcode_factory = None
        self.active_and_status_colour_hex_map = {
            (False, py_trees_msgs.Behaviour.INVALID): '#e4e4e4',
            (True, py_trees_msgs.Behaviour.INVALID): '#e4e4e4',
            (False, py_trees_msgs.Behaviour.RUNNING): '#9696c8',
            (True, py_trees_msgs.Behaviour.RUNNING): '#0000ff',
            (False, py_trees_msgs.Behaviour.FAILURE): '#c89696',
            (True, py_trees_msgs.Behaviour.FAILURE): '#ff0000',
            (False, py_trees_msgs.Behaviour.SUCCESS): '#96c896',
            (True, py_trees_msgs.Behaviour.SUCCESS): '#00ff00',
        }
        self.active_and_status_colour_tuple_map = {
            (False, py_trees_msgs.Behaviour.INVALID): (228, 228, 228),
            (True, py_trees_msgs.Behaviour.INVALID): (228, 228, 228),
            (False, py_trees_msgs.Behaviour.RUNNING): (150, 150, 200),
            (True, py_trees_msgs.Behaviour.RUNNING): (0, 0, 255),
            (False, py_trees_msgs.Behaviour.FAILURE): (200, 150, 150),
            (True, py_trees_msgs.Behaviour.FAILURE): (255, 0, 0),
            (False, py_trees_msgs.Behaviour.SUCCESS): (150, 200, 150),
            (True, py_trees_msgs.Behaviour.SUCCESS): (0, 255, 0),
        }

    def generate_dotcode(self,
                         dotcode_factory,
                         timer=rospy.Time,
                         behaviours=None,
                         timestamp=None,
                         rank='same',   # None, same, min, max, source, sink
                         ranksep=0.2,   # vertical distance between layers
                         rankdir='TB',  # direction of layout (TB top > bottom, LR left > right)
                         force_refresh=False):
        """
        :param py_trees_msgs.Behaviour[] behaviours:
        :param force_refresh: if False, may return same dotcode as last time
        """
        if self.firstcall is True:
            self.firstcall = False
            force_refresh = True

        drawing_args = {
            'dotcode_factory': dotcode_factory,
            "rank": rank,
            "rankdir": rankdir,
            "ranksep": ranksep}

        selection_changed = False
        if self.last_drawargs != drawing_args:
            selection_changed = True
            self.last_drawargs = drawing_args

            self.dotcode_factory = dotcode_factory
            self.rank = rank
            self.rankdir = rankdir
            self.ranksep = ranksep

        self.graph = self.generate(behaviours, timestamp)
        self.dotcode = self.dotcode_factory.create_dot(self.graph)
        return self.dotcode

    def type_to_shape(self, behaviour_type):
        """
        qt_dotgraph.node_item only supports drawing in qt of two
        shapes - box, ellipse.
        """
        if behaviour_type == py_trees_msgs.Behaviour.BEHAVIOUR:
            return 'ellipse'
        elif behaviour_type == py_trees_msgs.Behaviour.SEQUENCE:
            return 'box'
        elif behaviour_type == py_trees_msgs.Behaviour.SELECTOR:
            return 'ellipse'
        else:
            return 'box'

    def type_to_colour(self, behaviour_type):
        if behaviour_type == py_trees_msgs.Behaviour.BEHAVIOUR:
            return None
        elif behaviour_type == py_trees_msgs.Behaviour.SEQUENCE:
            return '#ff9900'
        elif behaviour_type == py_trees_msgs.Behaviour.SELECTOR:
            return '#808080'
        else:
            return None

    def type_to_string(self, behaviour_type):
        if behaviour_type == py_trees_msgs.Behaviour.BEHAVIOUR:
            return 'Behaviour'
        elif behaviour_type == py_trees_msgs.Behaviour.SEQUENCE:
            return 'Sequence'
        elif behaviour_type == py_trees_msgs.Behaviour.SELECTOR:
            return 'Selector'
        else:
            return None

    def status_to_string(self, behaviour_status):
        if behaviour_status == py_trees_msgs.Behaviour.INVALID:
            return 'Invalid'
        elif behaviour_status == py_trees_msgs.Behaviour.RUNNING:
            return 'Running'
        elif behaviour_status == py_trees_msgs.Behaviour.FAILURE:
            return 'Failure'
        elif behaviour_status == py_trees_msgs.Behaviour.SUCCESS:
            return 'Success'
        else:
            return None

    def behaviour_to_tooltip_string(self, behaviour):
        to_display = ['class_name', 'type', 'status', 'message']  # should be static
        string = ''

        for attr in to_display:
            if attr == 'type':
                value = self.type_to_string(getattr(behaviour, attr))
            elif attr == 'status':
                value = self.status_to_string(getattr(behaviour, attr))
            else:
                value = str(getattr(behaviour, attr))

            value = "<i>empty</i>" if not value else value

            string += '<b>' + attr.replace('_', ' ').title() + ':</b> ' + value + "<br>"

        return "\"" + string + "\""

    def generate(self, data, timestamp):
        """
        :param py_trees_msgs.Behaviour[] data:
        :param ??? timestamp:
        """
        graph = self.dotcode_factory.get_graph(rank=self.rank,
                                               rankdir=self.rankdir,
                                               ranksep=self.ranksep)

        if len(data) == 0:
            self.dotcode_factory.add_node_to_graph(graph, 'No behaviour data received')
            return graph

        behaviour_dict_by_id = {}
        for behaviour in data:
            behaviour_dict_by_id[behaviour.own_id] = behaviour
        # first, add nodes to the graph, along with some cached information to
        # make it easy to create the coloured edges on the second pass
        states = {}
        for behaviour in data:
            self.dotcode_factory.add_node_to_graph(graph,
                                                   str(behaviour.own_id),
                                                   nodelabel=behaviour.name,
                                                   shape=self.type_to_shape(behaviour.type),
                                                   color=self.active_and_status_colour_hex_map[(behaviour.is_active, behaviour.status)],
                                                   tooltip=self.behaviour_to_tooltip_string(behaviour))
            states[unique_id.fromMsg(behaviour.own_id)] = (behaviour.is_active, behaviour.status)

        for behaviour in data:
            for child_id in behaviour.child_ids:
                # edge colour is set using integer tuples, not hexes
                try:
                    (is_active, status) = states[unique_id.fromMsg(child_id)]
                except KeyError:
                    # the child isn't part of the 'visible' tree
                    continue
                edge_colour = self.active_and_status_colour_tuple_map[(is_active, status)]
                self.dotcode_factory.add_edge_to_graph(graph,
                                                       str(behaviour.own_id),
                                                       str(child_id),
                                                       color=edge_colour)

        return graph

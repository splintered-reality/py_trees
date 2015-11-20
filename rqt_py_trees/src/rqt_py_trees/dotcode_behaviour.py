# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from __future__ import with_statement, print_function

import time
import rospy
import yaml
import uuid_msgs.msg as uuid_msgs
import py_trees.msg as py_trees_msgs

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

    def generate_dotcode(self,
                         dotcode_factory,
                         timer=rospy.Time,
                         tree=None,
                         rank='same',   # None, same, min, max, source, sink
                         ranksep=0.2,   # vertical distance between layers
                         rankdir='TB',  # direction of layout (TB top > bottom, LR left > right)
                         force_refresh=False):
        """
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

        self.graph = self.generate(tree.behaviours, tree.header.stamp)
        self.dotcode = self.dotcode_factory.create_dot(self.graph)

        return self.dotcode

    def type_to_shape(self, behaviour_type):
        if behaviour_type == py_trees_msgs.Behaviour.BEHAVIOUR:
            return 'box'
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

    def status_to_colour(self, behaviour_status):
        if behaviour_status == py_trees_msgs.Behaviour.INVALID:
            return '#e4e4e4'
        elif behaviour_status == py_trees_msgs.Behaviour.RUNNING:
            return '#000000'
        elif behaviour_status == py_trees_msgs.Behaviour.FAILURE:
            return '#ff0000'
        elif behaviour_status == py_trees_msgs.Behaviour.SUCCESS:
            return '#00ff00'
        else:
            return None
        
    def generate(self, data, timestamp):
        graph = self.dotcode_factory.get_graph(rank=self.rank,
                                               rankdir=self.rankdir,
                                               ranksep=self.ranksep)

        if len(data) == 0:
            self.dotcode_factory.add_node_to_graph(graph, 'No behaviour data received')
            return graph

        root = uuid_msgs.UniqueID() # the root has an empty UUID first, add
        # nodes to the graph, and create a dict containing IDs and the current
        # state of each behaviour. We use this on the second pass to colour edges
        states = {}
        for behaviour in data:
            self.dotcode_factory.add_node_to_graph(graph,
                                                   str(behaviour.own_id),
                                                   nodelabel=behaviour.name,
                                                   shape=self.type_to_shape(behaviour.type),
                                                   color=self.status_to_colour(behaviour.status) or self.type_to_colour(behaviour.type))
            states[str(behaviour.own_id)] = behaviour.status

        for behaviour in data:
            for child_id in behaviour.child_ids:
                # edge colour is set differently for some reason
                edge_color = (224,224,224)
                state = states[str(child_id)]
                if state == py_trees_msgs.Behaviour.RUNNING:
                    edge_color = (0,0,0)
                elif state == py_trees_msgs.Behaviour.SUCCESS:
                    edge_color = (0,255,0)
                elif state == py_trees_msgs.Behaviour.FAILURE:
                    edge_color = (255,0,0)
                    
                self.dotcode_factory.add_edge_to_graph(graph, 
                                                       str(behaviour.own_id),
                                                       str(child_id),
                                                       color=edge_color)

        #     if not tf_frame_values['parent'] in data:
        #         root = tf_frame_values['parent']
        #     self.dotcode_factory.add_node_to_graph(graph,
        #                                            tf_frame_values['parent'],
        #                                            shape='ellipse')

        #     edge_label= '"Broadcaster: %s\\n' % str(tf_frame_values['broadcaster'])
        #     edge_label += 'Average rate: %s\\n' % str(tf_frame_values['rate'])
        #     edge_label += 'Buffer length: %s\\n' % str(tf_frame_values['buffer_length'])
        #     edge_label += 'Most recent transform: %s\\n' % str(tf_frame_values['most_recent_transform'])
        #     edge_label += 'Oldest transform: %s"' % str(tf_frame_values['oldest_transform'])
        #     self.dotcode_factory.add_edge_to_graph(graph,
        #                                            tf_frame_values['parent'],
        #                                            frame_dict,
        #                                            label=edge_label)

        # # create legend before root node
        # legend_label = '"Recorded at time: %s"' % str(timestamp)
        # self.dotcode_factory.add_node_to_graph(graph, legend_label)
        # self.dotcode_factory.add_edge_to_graph(graph,
        #                                        legend_label,
        #                                        root,
        #                                        style='invis')

        return graph

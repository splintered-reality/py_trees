#
# License: Yujin
#
##############################################################################
# Description
##############################################################################

"""
.. module:: pygraphvizfactory
   :platform: Unix
   :synopsis: Repackaging of the limiting ROS qt_dotgraph.pygraphvizfactory module.

Oh my spaghettified magnificence,
Bless my noggin with a tickle from your noodly appendages!

"""

##############################################################################
# Imports
##############################################################################

import pygraphviz

##############################################################################
# Classes
##############################################################################


# Reference implementation for a dotcode factory
class PygraphvizFactory():

    def __init__(self):
        pass

    def get_graph(self, graph_type='digraph', rank='same', simplify=True, rankdir='TB', ranksep=0.2, compound=True):
        graph = pygraphviz.AGraph(directed=(graph_type == 'digraph'), ranksep=ranksep, rankdir=rankdir, rank=rank, compound=True, simplify=simplify)
        return graph

    def add_node_to_graph(self,
                          graph,
                          nodename,
                          nodelabel=None,
                          shape='box',
                          color=None,
                          url=None,
                          tooltip=None):
        """
        creates a node item for this factory, adds it to the graph.
        Node name can vary from label but must always be same for the same node label
        """
        if nodename is None or nodename == '':
            raise ValueError('Empty Node name')
        if nodelabel is None:
            nodelabel = nodename

        kwargs = {}
        if tooltip is not None:
            kwargs['tooltip'] = tooltip
        if color is not None:
            kwargs['color'] = color

        graph.add_node(nodename, label=str(nodelabel), shape=shape, url=url, **kwargs)

    def add_subgraph_to_graph(self,
                              graph,
                              subgraphlabel,
                              rank='same',
                              simplify=True,
                              rankdir='TB',
                              ranksep=0.2,
                              compound=True,
                              color=None,
                              shape='box',
                              style='bold'):
        """
        creates a cluster subgraph  item for this factory, adds it to the graph.
        cluster name can vary from label but must always be same for the same node label.
        Most layouters require cluster names to start with cluster.
        """
        if subgraphlabel is None or subgraphlabel == '':
            raise ValueError('Empty subgraph label')

        sg = graph.add_subgraph(name="cluster_%s" % subgraphlabel, ranksep=ranksep, rankdir=rankdir, rank=rank, compound=compound, label=str(subgraphlabel), style=style, color=color)

        return sg

    def add_edge_to_graph(self, graph, nodename1, nodename2, label=None, url=None, simplify=True, style=None, penwidth=1, color=None):
        kwargs = {'url': url}
        if label is not None:
            kwargs['label'] = label
        if style is not None:
            kwargs['style'] = style
        kwargs['penwidth'] = str(penwidth)
        if color is not None:
            kwargs['colorR'] = str(color[0])
            kwargs['colorG'] = str(color[1])
            kwargs['colorB'] = str(color[2])
        graph.add_edge(nodename1, nodename2, **kwargs)

    def create_dot(self, graph):
        graph.layout('dot')
        # sadly pygraphviz generates line wraps cutting between numbers
        return graph.string().replace("\\\n", "")

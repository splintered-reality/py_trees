#
# License: Yujin
#
##############################################################################
# Description
##############################################################################

"""
.. module:: pydotfactory
   :platform: Unix
   :synopsis: Repackaging of the limiting ROS qt_dotgraph.pydotfactory module.

Oh my spaghettified magnificence,
Bless my noggin with a tickle from your noodly appendages!

"""

##############################################################################
# Imports
##############################################################################

from distutils.version import LooseVersion
import urllib

import pydot
# work around for https://bugs.launchpad.net/ubuntu/+source/pydot/+bug/1321135
import pyparsing
pyparsing._noncomma = "".join([c for c in pyparsing.printables if c != ","])

##############################################################################
# Classes
##############################################################################


# Reference implementation for a dotcode factory
class PydotFactory():

    def __init__(self):
        pass

    def escape_label(self, name):
        if name in ['graph', 'subgraph', 'node', 'edge']:
            ret = "%s_" % name
        else:
            ret = name
        return ret

    def escape_name(self, name):
        ret = urllib.quote(name.strip())
        ret = ret.replace('/', '_')
        ret = ret.replace('%', '_')
        ret = ret.replace('-', '_')
        return self.escape_label(ret)

    def get_graph(self, graph_type='digraph', rank='same', simplify=True, rankdir='TB', ranksep=0.2, compound=True):
        # Lucid version of pydot bugs with certain settings, not sure which version exactly fixes those
        if LooseVersion(pydot.__version__) > LooseVersion('1.0.10'):
            graph = pydot.Dot('graphname',
                              graph_type=graph_type,
                              rank=rank,
                              rankdir=rankdir,
                              simplify=simplify
                              )
            graph.set_ranksep(ranksep)
            graph.set_compound(compound)
        else:
            graph = pydot.Dot('graphname',
                              graph_type=graph_type,
                              rank=rank,
                              rankdir=rankdir)
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
        node = pydot.Node(self.escape_name(nodename))
        node.set_shape(shape)
        node.set_label(self.escape_label(nodelabel))
        if tooltip is not None:
            node.set_tooltip(tooltip)
        if url is not None:
            node.set_URL(self.escape_name(url))
        if color is not None:
            node.set_color(color)
        graph.add_node(node)

    def add_subgraph_to_graph(self,
                              graph,
                              subgraphname,
                              rank='same',
                              simplify=True,
                              rankdir='TB',
                              ranksep=0.2,
                              compound=True,
                              color=None,
                              shape='box',
                              style='bold',
                              subgraphlabel=None):
        """
        creates a cluster subgraph  item for this factory, adds it to the graph.
        cluster name can vary from label but must always be same for the same node label.
        Most layouters require cluster names to start with cluster.
        """
        if subgraphname is None or subgraphname == '':
            raise ValueError('Empty subgraph name')
        g = pydot.Cluster(self.escape_name(subgraphname), rank=rank, rankdir=rankdir, simplify=simplify, color=color)
        if 'set_style' in g.__dict__:
            g.set_style(style)
        if 'set_shape' in g.__dict__:
            g.set_shape(shape)
        if LooseVersion(pydot.__version__) > LooseVersion('1.0.10'):
            g.set_compound(compound)
            g.set_ranksep(ranksep)
        subgraphlabel = subgraphname if subgraphlabel is None else subgraphlabel
        subgraphlabel = self.escape_label(subgraphlabel)
        if subgraphlabel:
            g.set_label(subgraphlabel)
        if 'set_color' in g.__dict__:
            if color is not None:
                g.set_color(color)
        graph.add_subgraph(g)
        return g

    def add_edge_to_graph(self, graph, nodename1, nodename2, label=None, url=None, simplify=True, style=None, penwidth=1, color=None):
        if simplify and LooseVersion(pydot.__version__) < LooseVersion('1.0.10'):
            if graph.get_edge(self.escape_name(nodename1), self.escape_name(nodename2)) != []:
                return
        edge = pydot.Edge(self.escape_name(nodename1), self.escape_name(nodename2))
        if label is not None and label != '':
            edge.set_label(label)
        if url is not None:
            edge.set_URL(self.escape_name(url))
        if style is not None:
            edge.set_style(style)
        edge.obj_dict['attributes']['penwidth'] = str(penwidth)
        if color is not None:
            edge.obj_dict['attributes']['colorR'] = str(color[0])
            edge.obj_dict['attributes']['colorG'] = str(color[1])
            edge.obj_dict['attributes']['colorB'] = str(color[2])
        graph.add_edge(edge)

    def create_dot(self, graph):
        dot = graph.create_dot()
        # sadly pydot generates line wraps cutting between numbers
        return dot.replace("\\\n", "")

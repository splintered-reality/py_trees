#
# License: Yujin
#
##############################################################################
# Description
##############################################################################

"""
.. module:: dot_to_qt
   :platform: Unix
   :synopsis: Repackaging of the limiting ROS qt_dotgraph.dot_to_qt module.

Oh my spaghettified magnificence,
Bless my noggin with a tickle from your noodly appendages!

"""

##############################################################################
# Imports
##############################################################################

# import pydot
import pygraphviz
import pyparsing

# work around for https://bugs.launchpad.net/ubuntu/+source/pydot/+bug/1321135
pyparsing._noncomma = "".join([c for c in pyparsing.printables if c != ","])


from python_qt_binding.QtCore import QPointF, QRectF
from python_qt_binding.QtGui import QColor

from .edge_item import EdgeItem
from .node_item import NodeItem


POINTS_PER_INCH = 72

##############################################################################
# Support
##############################################################################


# hack required by pydot
def get_unquoted(item, name):
    value = item.get(name)
    if value is None:
        return None
    try:
        return value.strip('"\n"')
    except AttributeError:
        # not part of the string family
        return value

# approximately, for workarounds (TODO: get this from dotfile somehow)
LABEL_HEIGHT = 30

##############################################################################
# Classes
##############################################################################


# Class generating Qt Elements from doctcode
class DotToQtGenerator():

    def __init__(self):
        pass

    def getNodeItemForSubgraph(self, subgraph, highlight_level):
        # let pydot imitate pygraphviz api
        attr = {}
        for name in subgraph.get_attributes().iterkeys():
            value = get_unquoted(subgraph, name)
            attr[name] = value
        obj_dic = subgraph.__getattribute__("obj_dict")
        for name in obj_dic:
            if name not in ['nodes', 'attributes', 'parent_graph'] and obj_dic[name] is not None:
                attr[name] = get_unquoted(obj_dic, name)
            elif name == 'nodes':
                for key in obj_dic['nodes']['graph'][0]['attributes']:
                    attr[key] = get_unquoted(obj_dic['nodes']['graph'][0]['attributes'], key)
        subgraph.attr = attr

        bb = subgraph.attr.get('bb', None)
        if bb is None:
            # no bounding box
            return None
        bb = bb.strip('"').split(',')
        if len(bb) < 4:
            # bounding box is empty
            return None
        bounding_box = QRectF(0, 0, float(bb[2]) - float(bb[0]), float(bb[3]) - float(bb[1]))
        if 'lp' in subgraph.attr:
            label_pos = subgraph.attr['lp'].strip('"').split(',')
        else:
            label_pos = (float(bb[0]) + (float(bb[2]) - float(bb[0])) / 2, float(bb[1]) + (float(bb[3]) - float(bb[1])) - LABEL_HEIGHT / 2)
        bounding_box.moveCenter(QPointF(float(bb[0]) + (float(bb[2]) - float(bb[0])) / 2, -float(bb[1]) - (float(bb[3]) - float(bb[1])) / 2))
        name = subgraph.attr.get('label', '')
        color = QColor(subgraph.attr['color']) if 'color' in subgraph.attr else None
        subgraph_nodeitem = NodeItem(highlight_level,
                                     bounding_box,
                                     label=name,
                                     shape='box',
                                     color=color,
                                     label_pos=QPointF(float(label_pos[0]), -float(label_pos[1])))
        bounding_box = QRectF(bounding_box)
        # With clusters we have the problem that mouse hovers cannot
        # decide whether to be over the cluster or a subnode. Using
        # just the "title area" solves this. TODO: Maybe using a
        # border region would be even better (multiple RectF)
        bounding_box.setHeight(LABEL_HEIGHT)
        subgraph_nodeitem.set_hovershape(bounding_box)
        return subgraph_nodeitem

    def getNodeItemForNode(self, node, highlight_level):
        """
        returns a pyqt NodeItem object, or None in case of error or invisible style
        """
        # let pydot imitate pygraphviz api
#         attr = {}
#         for name in node.get_attributes().iterkeys():
#             value = get_unquoted(node, name)
#             attr[name] = value
#         obj_dic = node.__getattribute__("obj_dict")
#         for name in obj_dic:
#             if name not in ['attributes', 'parent_graph'] and obj_dic[name] is not None:
#                 attr[name] = get_unquoted(obj_dic, name)
#         node.attr = attr

        if 'style' in node.attr:
            if node.attr['style'] == 'invis':
                return None

        color = QColor(node.attr['color']) if 'color' in node.attr else None
        name = None
        if 'label' in node.attr:
            name = node.attr['label']
        elif 'name' in node.attr:
            name = node.attr['name']
        else:
            print("Error, no label defined for node with attr: %s" % node.attr)
            return None
        if name is None:
            # happens on Lucid pygraphviz version
            print("Error, label is None for node %s, pygraphviz version may be too old." % node)
        else:
            name = name.decode('string_escape')

        # decrease rect by one so that edges do not reach inside
        bb_width = len(name) / 5
        if 'width' in node.attr:
            bb_width = node.attr['width']
        bb_height = 1.0
        if 'width' in node.attr:
            bb_height = node.attr['height']
        bounding_box = QRectF(0, 0, POINTS_PER_INCH * float(bb_width) - 1.0, POINTS_PER_INCH * float(bb_height) - 1.0)
        pos = (0, 0)
        if 'pos' in node.attr:
            pos = node.attr['pos'].split(',')
        bounding_box.moveCenter(QPointF(float(pos[0]), -float(pos[1])))

        node_item = NodeItem(highlight_level=highlight_level,
                             bounding_box=bounding_box,
                             label=name,
                             shape=node.attr.get('shape', 'ellipse'),
                             color=color,
                             tooltip=node.attr.get('tooltip', None)
                             # parent=None,
                             # label_pos=None
                             )
        # node_item.setToolTip(self._generate_tool_tip(node.attr.get('URL', None)))
        return node_item

    def addEdgeItem(self, edge, nodes, edges, highlight_level, same_label_siblings=False):
        """
        adds EdgeItem by data in edge to edges
        :param same_label_siblings: if true, edges with same label will be considered siblings (collective highlighting)
        """
        # let pydot imitate pygraphviz api
#         attr = {}
#         for name in edge.get_attributes().iterkeys():
#             value = get_unquoted(edge, name)
#             attr[name] = value
#         edge.attr = attr

        if 'style' in edge.attr:
            if edge.attr['style'] == 'invis':
                return
        style = edge.attr.get('style', None)

        label = edge.attr.get('label', None)
        label_pos = edge.attr.get('lp', None)
        label_center = None
        if label_pos is not None:
            label_pos = label_pos.split(',')
            label_center = QPointF(float(label_pos[0]), -float(label_pos[1]))

        # try pydot, fallback for pygraphviz
        source_node = edge.get_source() if hasattr(edge, 'get_source') else edge[0]
        destination_node = edge.get_destination() if hasattr(edge, 'get_destination') else edge[1]

        # create edge with from-node and to-node
        edge_pos = "0,0"
        if 'pos' in edge.attr:
            edge_pos = edge.attr['pos']
        if label is not None:
            label = label.decode('string_escape')

        color = None
        if 'colorR' in edge.attr and 'colorG' in edge.attr and 'colorB' in edge.attr:
            r = edge.attr['colorR']
            g = edge.attr['colorG']
            b = edge.attr['colorB']
            color = QColor(float(r), float(g), float(b))

        edge_item = EdgeItem(highlight_level=highlight_level,
                             spline=edge_pos,
                             label_center=label_center,
                             label=label,
                             from_node=nodes[source_node],
                             to_node=nodes[destination_node],
                             penwidth=int(edge.attr['penwidth']),
                             edge_color=color,
                             style=style)

        if same_label_siblings:
            if label is None:
                # for sibling detection
                label = "%s_%s" % (source_node, destination_node)
            # symmetrically add all sibling edges with same label
            if label in edges:
                for sibling in edges[label]:
                    edge_item.add_sibling_edge(sibling)
                    sibling.add_sibling_edge(edge_item)

        if label not in edges:
            edges[label] = []
        edges[label].append(edge_item)

    def dotcode_to_qt_items(self, dotcode, highlight_level, same_label_siblings=False):
        """
        takes dotcode, runs layout, and creates qt items based on the dot layout.
        returns two dicts, one mapping node names to Node_Item, one mapping edge names to lists of Edge_Item
        :param same_label_siblings: if true, edges with same label will be considered siblings (collective highlighting)
        """
        # layout graph
        if dotcode is None:
            return {}, {}

        # pydot - this function is very buggy and expensive, quickly > 1s!
        # graph = pydot.graph_from_dot_data(dotcode.encode("ascii", "ignore"))
        # let pydot imitate pygraphviz api
        # graph.nodes_iter = graph.get_node_list
        # graph.edges_iter = graph.get_edge_list
        # graph.subgraphs_iter = graph.get_subgraph_list

        # pygraphviz
        graph = pygraphviz.AGraph(string=dotcode.encode("ascii", "ignore"), strict=False, directed=True)
        graph.layout(prog='dot')

        nodes = {}
        for subgraph in graph.subgraphs_iter():
            subgraph_nodeitem = self.getNodeItemForSubgraph(subgraph, highlight_level)
            # skip subgraphs with empty bounding boxes
            if subgraph_nodeitem is None:
                continue

            subgraph.nodes_iter = subgraph.get_node_list
            nodes[subgraph.get_name()] = subgraph_nodeitem
            for node in subgraph.nodes_iter():
                # hack required by pydot
                if node.get_name() in ('graph', 'node', 'empty'):
                    continue
                nodes[node.get_name()] = self.getNodeItemForNode(node, highlight_level)
        for node in graph.nodes_iter():
            # hack required by pydot
            if node.get_name() in ('graph', 'node', 'empty'):
                continue
            nodes[node.get_name()] = self.getNodeItemForNode(node, highlight_level)

        edges = {}

        for subgraph in graph.subgraphs_iter():
            subgraph.edges_iter = subgraph.get_edge_list
            for edge in subgraph.edges_iter():
                self.addEdgeItem(edge, nodes, edges,
                                 highlight_level=highlight_level,
                                 same_label_siblings=same_label_siblings)

        for edge in graph.edges_iter():
            self.addEdgeItem(edge, nodes, edges,
                             highlight_level=highlight_level,
                             same_label_siblings=same_label_siblings)

        return nodes, edges

    def graph_to_qt_items(self, graph, highlight_level, same_label_siblings=False):
        """
        takes a pydot/pygraphviz graph and creates qt items based on the dot layout.
        returns two dicts, one mapping node names to Node_Item, one mapping edge names to lists of Edge_Item
        :param same_label_siblings: if true, edges with same label will be considered siblings (collective highlighting)
        """
        if graph is None:
            return {}, {}

        # if pydot, let pydot imitate pygraphviz api
        # graph.nodes_iter = graph.get_node_list
        # graph.edges_iter = graph.get_edge_list
        # graph.subgraphs_iter = graph.get_subgraph_list

        nodes = {}
        for subgraph in graph.subgraphs_iter():
            subgraph_nodeitem = self.getNodeItemForSubgraph(subgraph, highlight_level)
            # skip subgraphs with empty bounding boxes
            if subgraph_nodeitem is None:
                continue

            subgraph.nodes_iter = subgraph.get_node_list
            nodes[subgraph.get_name()] = subgraph_nodeitem
            for node in subgraph.nodes_iter():
                # hack required by pydot
                if node.get_name() in ('graph', 'node', 'empty'):
                    continue
                nodes[node.get_name()] = self.getNodeItemForNode(node, highlight_level)
        for node in graph.nodes_iter():
            # hack required by pydot
            if node.get_name() in ('graph', 'node', 'empty'):
                continue
            nodes[node.get_name()] = self.getNodeItemForNode(node, highlight_level)

        edges = {}

        for subgraph in graph.subgraphs_iter():
            subgraph.edges_iter = subgraph.get_edge_list
            for edge in subgraph.edges_iter():
                self.addEdgeItem(edge, nodes, edges,
                                 highlight_level=highlight_level,
                                 same_label_siblings=same_label_siblings)

        for edge in graph.edges_iter():
            self.addEdgeItem(edge, nodes, edges,
                             highlight_level=highlight_level,
                             same_label_siblings=same_label_siblings)

        return nodes, edges

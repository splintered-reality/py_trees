#
# License: Yujin
#
##############################################################################
# Description
##############################################################################

"""
.. module:: node_item
   :platform: Unix
   :synopsis: Repackaging of the limiting ROS qt_dotgraph.node_item module.

Oh my spaghettified magnificence,
Bless my noggin with a tickle from your noodly appendages!

"""

##############################################################################
# Imports
##############################################################################

from python_qt_binding.QtCore import Qt
from python_qt_binding.QtGui import QBrush, QGraphicsEllipseItem, QGraphicsRectItem, QGraphicsSimpleTextItem, QPainterPath, QPen

from .graph_item import GraphItem

##############################################################################
# Classes
##############################################################################


class NodeItem(GraphItem):

    def __init__(self, highlight_level, bounding_box, label, shape, color=None, parent=None, label_pos=None, tooltip=None):
        super(NodeItem, self).__init__(highlight_level, parent)

        self._default_color = self._COLOR_BLACK if color is None else color
        self._brush = QBrush(self._default_color)
        self._label_pen = QPen()
        self._label_pen.setColor(self._default_color)
        self._label_pen.setJoinStyle(Qt.RoundJoin)
        self._ellipse_pen = QPen(self._label_pen)
        self._ellipse_pen.setWidth(1)

        self._incoming_edges = set()
        self._outgoing_edges = set()

        if shape == 'box':
            self._graphics_item = QGraphicsRectItem(bounding_box)
        else:
            self._graphics_item = QGraphicsEllipseItem(bounding_box)
        self.addToGroup(self._graphics_item)

        self._label = QGraphicsSimpleTextItem(label)
        label_rect = self._label.boundingRect()
        if label_pos is None:
            label_rect.moveCenter(bounding_box.center())
        else:
            label_rect.moveCenter(label_pos)
        self._label.setPos(label_rect.x(), label_rect.y())
        self.addToGroup(self._label)
        if tooltip is not None:
            self.setToolTip(tooltip)

        self.set_node_color()

        self.setAcceptHoverEvents(True)

        self.hovershape = None

    def set_hovershape(self, newhovershape):
        self.hovershape = newhovershape

    def shape(self):
        if self.hovershape is not None:
            path = QPainterPath()
            path.addRect(self.hovershape)
            return path
        else:
            return super(self.__class__, self).shape()

    def add_incoming_edge(self, edge):
        self._incoming_edges.add(edge)

    def add_outgoing_edge(self, edge):
        self._outgoing_edges.add(edge)

    def set_node_color(self, color=None):
        if color is None:
            color = self._default_color

        self._brush.setColor(color)
        self._ellipse_pen.setColor(color)
        self._label_pen.setColor(color)

        self._graphics_item.setPen(self._ellipse_pen)
        self._label.setBrush(self._brush)
        self._label.setPen(self._label_pen)

    def hoverEnterEvent(self, event):
        # hovered node item in red
        self.set_node_color(self._COLOR_RED)

        if self._highlight_level > 1:
            cyclic_edges = self._incoming_edges.intersection(self._outgoing_edges)
            # incoming edges in blue
            incoming_nodes = set()
            for incoming_edge in self._incoming_edges.difference(cyclic_edges):
                incoming_edge.set_node_color(self._COLOR_BLUE)
                if incoming_edge.from_node != self:
                    incoming_nodes.add(incoming_edge.from_node)
            # outgoing edges in green
            outgoing_nodes = set()
            for outgoing_edge in self._outgoing_edges.difference(cyclic_edges):
                outgoing_edge.set_node_color(self._COLOR_GREEN)
                if outgoing_edge.to_node != self:
                    outgoing_nodes.add(outgoing_edge.to_node)
            # incoming/outgoing edges in teal
            for edge in cyclic_edges:
                edge.set_node_color(self._COLOR_TEAL)

            if self._highlight_level > 2:
                cyclic_nodes = incoming_nodes.intersection(outgoing_nodes)
                # incoming nodes in blue
                for incoming_node in incoming_nodes.difference(cyclic_nodes):
                    incoming_node.set_node_color(self._COLOR_BLUE)
                # outgoing nodes in green
                for outgoing_node in outgoing_nodes.difference(cyclic_nodes):
                    outgoing_node.set_node_color(self._COLOR_GREEN)
                # incoming/outgoing nodes in teal
                for node in cyclic_nodes:
                    node.set_node_color(self._COLOR_TEAL)

    def hoverLeaveEvent(self, event):
        self.set_node_color()
        if self._highlight_level > 1:
            for incoming_edge in self._incoming_edges:
                incoming_edge.set_node_color()
                if self._highlight_level > 2 and incoming_edge.from_node != self:
                    incoming_edge.from_node.set_node_color()
            for outgoing_edge in self._outgoing_edges:
                outgoing_edge.set_node_color()
                if self._highlight_level > 2 and outgoing_edge.to_node != self:
                    outgoing_edge.to_node.set_node_color()

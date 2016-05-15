#
# License: Yujin
#
##############################################################################
# Description
##############################################################################

"""
.. module:: graph_item
   :platform: Unix
   :synopsis: Repackaging of the limiting ROS qt_dotgraph.graph_item module.

Oh my spaghettified magnificence,
Bless my noggin with a tickle from your noodly appendages!

"""

##############################################################################
# Imports
##############################################################################

from python_qt_binding.QtGui import QColor, QGraphicsItemGroup

##############################################################################
# Classes
##############################################################################


class GraphItem(QGraphicsItemGroup):

    _COLOR_BLACK = QColor(0, 0, 0)
    _COLOR_BLUE = QColor(0, 0, 204)
    _COLOR_GREEN = QColor(0, 170, 0)
    _COLOR_ORANGE = QColor(255, 165, 0)
    _COLOR_RED = QColor(255, 0, 0)
    _COLOR_TEAL = QColor(0, 170, 170)

    def __init__(self, highlight_level, parent=None):
        super(GraphItem, self).__init__(parent)
        self._highlight_level = highlight_level

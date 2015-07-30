#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/yujinrobot/gopher_crazy_hospital/license/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
.. module:: display
   :platform: Unix
   :synopsis: Render and write the behaviour tree visually to file or stdout.

Various graph drawing tools.

----

"""

##############################################################################
# Imports
##############################################################################

import pygraphviz as pgv
from pygraph.classes.digraph import digraph
from pygraph.algorithms.searching import breadth_first_search
from pygraph.readwrite.dot import write
import gv


##############################################################################
# Behaviour
##############################################################################


def print_ascii_tree(printTree, indent=0):
    """
    Print the ASCII representation of an act tree.
    :param tree: The root of an act tree
    :param indent: the number of characters to indent the tree
    :return: nothing
    """
    for child in printTree.children:
        print "    " * indent, "-->", child.name

        if child.children != []:
            print_ascii_tree(child, indent + 1)


def print_dot_tree(root):
    """
        Print an output compatible with the DOT synatax and Graphiz
    """
    gr = pgv.AGraph(rotate='0', bgcolor='lightyellow')
    gr.node_attr['fontsize'] = '9'

    def add_edges(root):
        for c in root.children:
            gr.add_edge((root.name, c.name))
            if c.children != []:
                add_edges(c)

    add_edges(root)

    st, unused_order = breadth_first_search(gr, root=root.name)

    gst = digraph()
    gst.add_spanning_tree(st)

    dot = write(gst)
    gvv = gv.readstring(dot)

    gv.layout(gvv, 'dot')
    gv.render(gvv, 'png', 'tree.png')
    gv.render(gvv, 'svg', 'tree.svg')
    gv.write(gvv, 'tree.dot')

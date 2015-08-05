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

import pydot
from .composites import Sequence, Selector

##############################################################################
# Behaviour
##############################################################################


def print_ascii_tree(tree, indent=0):
    """
    Print the ASCII representation of a behaviour tree.

    :param tree: the root of the tree, or subtree you want to show
    :param indent: the number of characters to indent the tree
    :return: nothing
    """
    if indent == 0:
        print("%s" % tree.name)
    for child in tree.children:
        print "    " * indent, "-->", child.name

        if child.children != []:
            print_ascii_tree(child, indent + 1)


def render_dot_tree(root):
    def get_node_attributes(node):
        if isinstance(node, Selector):
            return ('box', 'cyan')
        elif isinstance(node, Sequence):
            return ('box', 'green')
        else:
            return ('ellipse', 'gray')

    fontsize = 11
    graph = pydot.Dot(graph_type='digraph')
    graph.set_name(root.name)
    (unused_node_shape, node_colour) = get_node_attributes(root)
    node_root = pydot.Node(root.name, shape="house", style="filled", fillcolor=node_colour, fontsize=fontsize)
    graph.add_node(node_root)

    def add_edges(root):
        for c in root.children:
            (node_shape, node_colour) = get_node_attributes(c)
            node = pydot.Node(c.name, shape=node_shape, style="filled", fillcolor=node_colour, fontsize=fontsize)
            graph.add_node(node)
            edge = pydot.Edge(root.name, c.name)
            graph.add_edge(edge)
            if c.children != []:
                add_edges(c)

    add_edges(root)
    name = root.name.lower()
    print("Writing %s.dot/svg/png" % name)
    graph.write(name + '.dot')
    graph.write_png(name + '.png')
    graph.write_svg(name + '.svg')

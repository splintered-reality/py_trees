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


def generate_ascii_tree(tree, indent=0):
    """
    Generate the ascii tree (worker function).

    :param tree: the root of the tree, or subtree you want to show
    :param indent: the number of characters to indent the tree
    :return: a generator that yields ascii representations of nodes one by one
    """
    if indent == 0:
        yield "%s" % tree.name
    for child in tree.children:
        yield "    " * indent + "-->" + child.name
        if child.children != []:
            for line in generate_ascii_tree(child, indent + 1):
                yield line


def ascii_tree(tree, indent=0):
    """
    Build the ascii tree representation as a string for redirecting
    to elsewhere other than stdout (e.g. ros publisher)

    :param tree: the root of the tree, or subtree you want to show
    :param indent: the number of characters to indent the tree
    :return: ascii_tree as a string

    """
    s = ""
    for line in generate_ascii_tree(tree, indent):
        s += "%s\n" % line
    return s


def print_ascii_tree(tree, indent=0):
    """
    Print the ASCII representation of a behaviour tree.

    :param tree: the root of the tree, or subtree you want to show
    :param indent: the number of characters to indent the tree
    :return: nothing
    """
    for line in generate_ascii_tree(tree, indent):
        print("%s" % line)


def generate_pydot_graph(root):
    """
    Render the dot tree to files - _name_.dot, .svg, .png.
    These files will be created in the present working directory.

    :param root: the root of the tree, or subtree you want to generate
    :return: the dot graph as a pydot.Dot graph
    """
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
    return graph


def stringify_dot_tree(root):
    """
    Generate dot tree graphs and return a string representation of the dot graph.

    :param root: the root of the tree, or subtree you want to show
    :return: dot graph as a string
    """
    graph = generate_pydot_graph(root)
    return graph.to_string()


def render_dot_tree(root):
    """
    Render the dot tree to files - _name_.dot, .svg, .png.
    These files will be created in the present working directory.

    :param root: the root of the tree, or subtree you want to show
    """
    graph = generate_pydot_graph(root)
    name = root.name.lower()
    print("Writing %s.dot/svg/png" % name)
    graph.write(name + '.dot')
    graph.write_png(name + '.png')
    graph.write_svg(name + '.svg')

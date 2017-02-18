#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/stonier/py_trees_suite/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
.. module:: display
   :platform: Unix
   :synopsis: Render and write the behaviour tree visually to file or stdout.

Various graph drawing tools.
"""

##############################################################################
# Imports
##############################################################################

import pydot

from . import common
from . import console

from .composites import Sequence, Selector, Parallel, Chooser
from .common import Status

##############################################################################
# Behaviour
##############################################################################

# hide from public exposure
_behaviour_status_to_ascii = {
    Status.SUCCESS: console.green + u'\u2713' + console.reset,
    Status.FAILURE: console.red + "x" + console.reset,
    Status.INVALID: console.yellow + "-" + console.reset,
    Status.RUNNING: console.blue + "*" + console.reset
}


def _generate_ascii_tree(tree, indent=0, snapshot_information=None):
    """
    Generator for spinning out the ascii tree.

    :param tree: the root of the tree, or subtree you want to show
    :param indent: the number of characters to indent the tree
    :param snapshot_information: a visitor recording information about the tree runtime (e.g. BehaviourTree.SnapshotVisitor)
    :return: a generator that yields ascii representations of nodes one by one
    """
    nodes = {} if snapshot_information is None else snapshot_information.nodes
    previously_running_nodes = [] if snapshot_information is None else snapshot_information.previously_running_nodes
    running_nodes = [] if snapshot_information is None else snapshot_information.running_nodes

    if indent == 0:
        if tree.id in nodes:
            yield "%s [%s]" % (tree.name, _behaviour_status_to_ascii[nodes[tree.id]])
        elif tree.id in previously_running_nodes and tree.id not in running_nodes:
            yield "%s" % tree.name + " [" + console.yellow + "-" + console.reset + "]"
        else:
            yield "%s" % tree.name
    for child in tree.children:
        bullet = "--> "
        if isinstance(child, Sequence):
            bullet = "[-] "
        elif isinstance(child, Selector):
            bullet = "(-) "
        elif isinstance(child, Parallel):
            bullet = "(o) "
        if child.id in nodes:
            message = "" if not child.feedback_message else " -- " + child.feedback_message
            yield "    " * indent + bullet + child.name + " [%s]" % _behaviour_status_to_ascii[nodes[child.id]] + message
        elif child.id in previously_running_nodes and child.id not in running_nodes:
            yield "    " * indent + bullet + child.name + " [" + console.yellow + "-" + console.reset + "]"
        else:
            yield "    " * indent + bullet + child.name
        if child.children != []:
            for line in _generate_ascii_tree(child, indent + 1, snapshot_information):
                yield line


def ascii_tree(tree, indent=0, snapshot_information=None):
    """
    Build the ascii tree representation as a string for redirecting
    to elsewhere other than stdout (e.g. ros publisher)

    :param tree: the root of the tree, or subtree you want to show
    :param indent: the number of characters to indent the tree
    :param snapshot_information: a visitor recording information about the tree runtime (e.g. BehaviourTree.SnapshotVisitor)
    :return: ascii_tree as a string
    """
    s = ""
    for line in _generate_ascii_tree(tree, indent, snapshot_information):
        s += "%s\n" % line
    return s


def print_ascii_tree(tree, indent=0, show_status=False):
    """
    Print the ASCII representation of a behaviour tree.

    :param tree: the root of the tree, or subtree you want to show
    :param indent: the number of characters to indent the tree
    :param show_status: show the status of every element at this instant
    :return: nothing
    """
    class InstantSnapshot(object):
        def __init__(self, tree):
            self.nodes = {}
            self.previously_running_nodes = []
            self.running_nodes = []
            for node in tree.iterate():
                self.nodes[node.id] = node.status
                self.running_nodes.append(node.id)
            self.previously_running_nodes = self.running_nodes
    snapshot_information = InstantSnapshot(tree) if show_status else None
    for line in _generate_ascii_tree(tree, indent, snapshot_information):
        print("%s" % line)


def generate_pydot_graph(root, visibility_level):
    """
    Generate the pydot graph - this is usually the first step in
    rendering the tree to file. See also :py:func:`render_dot_tree`.

    :param root: the root of the tree, or subtree you want to generate
    :param common.BlackBoxLevel blackbox_level: collapse subtrees at or under this level
    :return: the dot graph as a pydot.Dot graph
    """
    def get_node_attributes(node, visibility_level):
        blackbox_font_colours = {common.BlackBoxLevel.DETAIL: "dodgerblue",
                                 common.BlackBoxLevel.COMPONENT: "lawngreen",
                                 common.BlackBoxLevel.BIG_PICTURE: "white"
                                 }
        if isinstance(node, Chooser):
            attributes = ('doubleoctagon', 'cyan', 'black')  # octagon
        elif isinstance(node, Selector):
            attributes = ('octagon', 'cyan', 'black')  # octagon
        elif isinstance(node, Sequence):
            attributes = ('box', 'orange', 'black')
        elif isinstance(node, Parallel):
            attributes = ('note', 'gold', 'black')
        elif node.children != []:
            attributes = ('ellipse', 'ghostwhite', 'black')  # encapsulating behaviour (e.g. wait)
        else:
            attributes = ('ellipse', 'gray', 'black')
        if node.blackbox_level != common.BlackBoxLevel.NOT_A_BLACKBOX:
            attributes = (attributes[0], 'gray20', blackbox_font_colours[node.blackbox_level])
        return attributes

    fontsize = 11
    graph = pydot.Dot(graph_type='digraph')
    graph.set_name(root.name.lower().replace(" ", "_"))
    # fonts: helvetica, times-bold, arial (times-roman is the default, but this helps some viewers, like kgraphviewer)
    graph.set_graph_defaults(fontname='times-roman')
    graph.set_node_defaults(fontname='times-roman')
    graph.set_edge_defaults(fontname='times-roman')
    (node_shape, node_colour, node_font_colour) = get_node_attributes(root, visibility_level)
    node_root = pydot.Node(root.name, shape=node_shape, style="filled", fillcolor=node_colour, fontsize=fontsize, fontcolor=node_font_colour)
    graph.add_node(node_root)
    names = [root.name]

    def add_edges(root, root_dot_name, visibility_level):
        if visibility_level < root.blackbox_level:
            for c in root.children:
                (node_shape, node_colour, node_font_colour) = get_node_attributes(c, visibility_level)
                proposed_dot_name = c.name
                while proposed_dot_name in names:
                    proposed_dot_name = proposed_dot_name + "*"
                names.append(proposed_dot_name)
                node = pydot.Node(proposed_dot_name, shape=node_shape, style="filled", fillcolor=node_colour, fontsize=fontsize, fontcolor=node_font_colour)
                graph.add_node(node)
                edge = pydot.Edge(root_dot_name, proposed_dot_name)
                graph.add_edge(edge)
                if c.children != []:
                    add_edges(c, proposed_dot_name, visibility_level)

    add_edges(root, root.name, visibility_level)
    return graph


def stringify_dot_tree(root):
    """
    Generate dot tree graphs and return a string representation
    of the dot graph.

    :param root: the root of the tree, or subtree you want to show
    :return: dot graph as a string
    """
    graph = generate_pydot_graph(root, visibility_level=common.VisibilityLevel.DETAIL)
    return graph.to_string()


def render_dot_tree(root, visibility_level=common.VisibilityLevel.DETAIL):
    """
    Render the dot tree to .dot, .svg, .png. files in the current
    working directory. These will be named with the root behaviour name.

    :param Behaviour root: the root of the tree, or subtree you want to show
    :param int depth: the number of blackbox levels to descend into (-1 shows full tree)
    """
    graph = generate_pydot_graph(root, visibility_level)
    name = root.name.lower().replace(" ", "_")
    print("Writing %s.dot/svg/png" % name)
    graph.write(name + '.dot')
    graph.write_png(name + '.png')
    graph.write_svg(name + '.svg')

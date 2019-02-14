#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/stonier/py_trees/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
Behaviour trees are significantly easier to design, monitor and debug
with visualisations. Py Trees does provide minimal assistance to render
trees to various simple output formats. Currently this includes dot graphs,
strings or stdout.
"""

##############################################################################
# Imports
##############################################################################

import pydot

from . import behaviour
from . import common
from . import composites
from . import console
from . import decorators
from . import utilities


##############################################################################
# Methods
##############################################################################

@utilities.static_variables(
    type_to_unicode={
        composites.Sequence: "[-] ",
        composites.Selector: "[o] ",
        composites.Parallel: "[" + u'\u2016' + "] ",
        decorators.Decorator: "-^- ",
        behaviour.Behaviour: "--> "
    }
)
def ascii_bullet(node):
    """
    Generate a text bullet for the specified behaviour's type.

    Args:
        node (:class:`~py_trees.behaviour.Behaviour`): convert this behaviour's type to text

    Returns:
        :obj:`str`): the text bullet
    """
    for behaviour_type in [composites.Sequence,
                           composites.Selector,
                           composites.Parallel,
                           decorators.Decorator]:
        if isinstance(node, behaviour_type):
            return ascii_bullet.type_to_unicode[behaviour_type]
    return ascii_bullet.type_to_unicode[behaviour.Behaviour]


@utilities.static_variables(
    status_to_unicode={
        common.Status.SUCCESS: console.green + u'\u2713' + console.reset,
        common.Status.FAILURE: console.red + u'\u2715' + console.reset,
        common.Status.INVALID: console.yellow + u'-' + console.reset,
        common.Status.RUNNING: console.blue + u'*' + console.reset
    }
)
def ascii_check_mark(status):
    """
    Generate a text check mark for the specified status.

    Args:
        status (:class:`~py_trees.common.Status`): convert this status to text

    Returns:
        :obj:`str`): the text check mark
    """
    return ascii_check_mark.status_to_unicode[status]


def _generate_ascii_tree(tree, indent=0, snapshot_information=None):
    """
    Generator for spinning out the ascii tree.

    Args:
        tree (:class:`~py_trees.behaviour.Behaviour`): the root of the tree, or subtree you want to show
        indent (:obj:`int`): the number of characters to indent the tree
        snapshot_information (:mod:`~py_trees.visitors`): a visitor that recorded information about a traversed tree (e.g. :class:`~py_trees.visitors.SnapshotVisitor`)

    Yields:
        :obj:`str`: a string with information about a single behaviour in the tree
    """

    nodes = {} if snapshot_information is None else snapshot_information.nodes
    previously_running_nodes = [] if snapshot_information is None else snapshot_information.previously_running_nodes
    running_nodes = [] if snapshot_information is None else snapshot_information.running_nodes
    if indent == 0:
        if tree.id in nodes:
            yield "%s [%s]" % (tree.name.replace('\n', ' '), ascii_check_mark(nodes[tree.id]))
        elif tree.id in previously_running_nodes and tree.id not in running_nodes:
            yield "%s" % tree.name.replace('\n', ' ') + " [" + console.yellow + "-" + console.reset + "]"
        else:
            yield "%s" % tree.name.replace('\n', ' ')
    for child in tree.children:
        bullet = ascii_bullet(child)
        if child.id in nodes:
            message = "" if not child.feedback_message else " -- " + child.feedback_message
            yield "    " * indent + bullet + child.name.replace('\n', ' ') + " [%s]" % ascii_check_mark(nodes[child.id]) + message
        elif child.id in previously_running_nodes and child.id not in running_nodes:
            yield "    " * indent + bullet + child.name.replace('\n', ' ') + " [" + console.yellow + "-" + console.reset + "]"
        else:
            yield "    " * indent + bullet + child.name.replace('\n', ' ')
        if child.children != []:
            for line in _generate_ascii_tree(child, indent + 1, snapshot_information):
                yield line


def ascii_tree(tree, indent=0, snapshot_information=None):
    """
    Build an ascii tree representation as a string for redirecting
    to elsewhere other than stdout. This can be the entire tree, or
    a recorded snapshot of the tree (i.e. just the part that was traversed).

    Args:
        tree (:class:`~py_trees.behaviour.Behaviour`): the root of the tree, or subtree you want to show
        indent (:obj:`int`): the number of characters to indent the tree
        snapshot_information (:mod:`~py_trees.visitors`): a visitor that recorded information about a traversed tree (e.g. :class:`~py_trees.visitors.SnapshotVisitor`)
        snapshot_information (:mod:`~py_trees.visitors`): a visitor that recorded information about a traversed tree (e.g. :class:`~py_trees.visitors.SnapshotVisitor`)

    Returns:
        :obj:`str`: an ascii tree (i.e. in string form)

    Examples:

        Use the :class:`~py_trees.visitors.SnapshotVisitor`
        and :class:`~py_trees.trees.BehaviourTree`
        to generate snapshot information at each tick and feed that to
        a post tick handler that will print the traversed ascii tree
        complete with status and feedback messages.

        .. image:: images/ascii_tree.png
            :width: 200px
            :align: right

        .. code-block:: python

            def post_tick_handler(snapshot_visitor, behaviour_tree):
                print(py_trees.display.ascii_tree(behaviour_tree.root,
                      snapshot_information=snapshot_visitor))

            root = py_trees.composites.Sequence("Sequence")
            for action in ["Action 1", "Action 2", "Action 3"]:
                b = py_trees.behaviours.Count(
                        name=action,
                        fail_until=0,
                        running_until=1,
                        success_until=10)
                root.add_child(b)
            behaviour_tree = py_trees.trees.BehaviourTree(root)
            snapshot_visitor = py_trees.visitors.SnapshotVisitor()
            behaviour_tree.add_post_tick_handler(
                functools.partial(post_tick_handler,
                                  snapshot_visitor))
            behaviour_tree.visitors.append(snapshot_visitor)

    """
    s = ""
    for line in _generate_ascii_tree(tree, indent, snapshot_information):
        s += "%s\n" % line
    return s


def print_ascii_tree(root, indent=0, show_status=False):
    """
    Print the ASCII representation of an entire behaviour tree.

    Args:
        root (:class:`~py_trees.behaviour.Behaviour`): the root of the tree, or subtree you want to show
        indent (:obj:`int`): the number of characters to indent the tree
        show_status (:obj:`bool`): additionally show feedback message and status of every element

    Examples:

        Render a simple tree in ascii format to stdout.

        .. image:: images/ascii_tree_simple.png
            :width: 100px
            :align: right

        .. code-block:: python

            root = py_trees.composites.Sequence("Sequence")
            for action in ["Action 1", "Action 2", "Action 3"]:
                b = py_trees.behaviours.Count(
                        name=action,
                        fail_until=0,
                        running_until=1,
                        success_until=10)
                root.add_child(b)
            py_trees.display.print_ascii_tree(root)

    .. tip::

        To additionally display status and feedbback message from every behaviour in the tree,
        simply set the :obj:`show_status` flag to True.
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
    snapshot_information = InstantSnapshot(root) if show_status else None
    for line in _generate_ascii_tree(root, indent, snapshot_information):
        print("%s" % line)


def generate_pydot_graph(root, visibility_level, collapse_decorators=False):
    """
    Generate the pydot graph - this is usually the first step in
    rendering the tree to file. See also :py:func:`render_dot_tree`.

    Args:
        root (:class:`~py_trees.behaviour.Behaviour`): the root of a tree, or subtree
        visibility_level (:class`~py_trees.common.VisibilityLevel`): collapse subtrees at or under this level
        collapse_decorators (:obj:`bool`): only show the decorator (not the child)

    Returns:
        pydot.Dot: graph
    """
    def get_node_attributes(node, visibility_level):
        blackbox_font_colours = {common.BlackBoxLevel.DETAIL: "dodgerblue",
                                 common.BlackBoxLevel.COMPONENT: "lawngreen",
                                 common.BlackBoxLevel.BIG_PICTURE: "white"
                                 }
        if isinstance(node, composites.Chooser):
            attributes = ('doubleoctagon', 'cyan', 'black')  # octagon
        elif isinstance(node, composites.Selector):
            attributes = ('octagon', 'cyan', 'black')  # octagon
        elif isinstance(node, composites.Sequence):
            attributes = ('box', 'orange', 'black')
        elif isinstance(node, composites.Parallel):
            attributes = ('parallelogram', 'gold', 'black')
        elif isinstance(node, decorators.Decorator):
            attributes = ('ellipse', 'ghostwhite', 'black')
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

    def add_edges(root, root_dot_name, visibility_level, collapse_decorators):
        if isinstance(root, decorators.Decorator) and collapse_decorators:
            return
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
                    add_edges(c, proposed_dot_name, visibility_level, collapse_decorators)

    add_edges(root, root.name, visibility_level, collapse_decorators)
    return graph


def stringify_dot_tree(root):
    """
    Generate dot tree graphs and return a string representation
    of the dot graph.

    Args:
        root (:class:`~py_trees.behaviour.Behaviour`): the root of a tree, or subtree

    Returns:
        :obj:`str`: dot graph as a string
    """
    graph = generate_pydot_graph(root, visibility_level=common.VisibilityLevel.DETAIL)
    return graph.to_string()


def render_dot_tree(root,
                    visibility_level=common.VisibilityLevel.DETAIL,
                    collapse_decorators=False,
                    name=None):
    """
    Render the dot tree to .dot, .svg, .png. files in the current
    working directory. These will be named with the root behaviour name.

    Args:
        root (:class:`~py_trees.behaviour.Behaviour`): the root of a tree, or subtree
        visibility_level (:class`~py_trees.common.VisibilityLevel`): collapse subtrees at or under this level
        collapse_decorators (:obj:`bool`): only show the decorator (not the child)
        name (:obj:`str`): name to use for the created files (defaults to the root behaviour name)

    Example:

        Render a simple tree to dot/svg/png file:

        .. graphviz:: dot/sequence.dot

        .. code-block:: python

            root = py_trees.composites.Sequence("Sequence")
            for job in ["Action 1", "Action 2", "Action 3"]:
                success_after_two = py_trees.behaviours.Count(name=job,
                                                              fail_until=0,
                                                              running_until=1,
                                                              success_until=10)
                root.add_child(success_after_two)
            py_trees.display.render_dot_tree(root)

    .. tip::

        A good practice is to provide a command line argument for optional rendering of a program so users
        can quickly visualise what tree the program will execute.
    """
    graph = generate_pydot_graph(root, visibility_level, collapse_decorators)
    filename_wo_extension = root.name.lower().replace(" ", "_") if name is None else name
    print("Writing %s.dot/svg/png" % filename_wo_extension)
    graph.write(filename_wo_extension + '.dot')
    graph.write_png(filename_wo_extension + '.png')
    graph.write_svg(filename_wo_extension + '.svg')

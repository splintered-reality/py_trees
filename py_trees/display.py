#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/splintered-reality/py_trees/devel/LICENSE
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

import os
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

unicode_symbols = {
    'space': ' ',
    'bold': console.bold,
    'bold_reset': console.reset,
    composites.Sequence: u'[-]',
    composites.Selector: u'[o]',
    composites.Parallel: u'[' + console.double_vertical_line + u']',
    decorators.Decorator: u'-^-',
    behaviour.Behaviour: u'-->',
    common.Status.SUCCESS: console.green + console.check_mark + console.reset,
    common.Status.FAILURE: console.red + console.multiplication_x + console.reset,
    common.Status.INVALID: console.yellow + u'-' + console.reset,
    common.Status.RUNNING: console.blue + u'*' + console.reset
}
"""Symbols for a unicode, escape sequence capable console."""

ascii_symbols = {
    'space': ' ',
    'bold': console.bold,
    'bold_reset': console.reset,
    composites.Sequence: "[-]",
    composites.Selector: "[o]",
    composites.Parallel: "[||]",
    decorators.Decorator: "-^-",
    behaviour.Behaviour: "-->",
    common.Status.SUCCESS: console.green + 'o' + console.reset,
    common.Status.FAILURE: console.red + 'x' + console.reset,
    common.Status.INVALID: console.yellow + '-' + console.reset,
    common.Status.RUNNING: console.blue + '*' + console.reset
}
"""Symbols for a non-unicode, non-escape sequence capable console."""
xhtml_symbols = {
    'space': '<text>&#xa0;</text>',  # &nbsp; is not valid xhtml, see http://www.fileformat.info/info/unicode/char/00a0/index.htm
    'bold': '<b>',
    'bold_reset': '</b>',
    composites.Sequence: '<text>[-]</text>',
    composites.Selector: '<text>[o]</text>',
    composites.Parallel: '<text style="color:green;">[&#x2016;]</text>',  # c.f. console.double_vertical_line
    decorators.Decorator: '<text>-^-</text>',
    behaviour.Behaviour: '<text>--></text>',
    common.Status.SUCCESS: '<text style="color:green;">&#x2713;</text>',  # c.f. console.check_mark
    common.Status.FAILURE: '<text style="color:red;">&#x2715;</text>',  # c.f. console.multiplication_x
    common.Status.INVALID: '<text style="color:darkgoldenrod;">-</text>',
    common.Status.RUNNING: '<text style="color:blue;">*</text>'
}
"""Symbols for embedding in html."""


def _generate_text_tree(
        root,
        show_status=False,
        visited={},
        previously_visited={},
        indent=0,
        symbols=None,
     ):
    """
    Generate a text tree utilising the specified symbol formatter.

    Args:
        root (:class:`~py_trees.behaviour.Behaviour`): the root of the tree, or subtree you want to show
        show_status (:obj:`bool`): always show status and feedback message (i.e. for every element,
            not just those visited)
        visited (dict): dictionary of (uuid.UUID) and status (:class:`~py_trees.common.Status`) pairs
            for behaviours visited on the current tick
        previously_visited (dict): dictionary of behaviour id/status pairs from the previous tree tick
        indent (:obj:`int`): the number of characters to indent the tree
        symbols (dict, optional): dictates formatting style
            (one of :data:`py_trees.display.unicode_symbols` || :data:`py_trees.display.ascii_symbols` || :data:`py_trees.display.xhtml_symbols`),
            defaults to unicode if stdout supports it, ascii otherwise

    Returns:
        :obj:`str`: a text-based representation of the behaviour tree

    .. seealso:: :meth:`py_trees.display.ascii_tree`, :meth:`py_trees.display.unicode_tree`, :meth:`py_trees.display.xhtml_tree`
    """
    # default to unicode if stdout supports it, ascii otherwise
    if symbols is None:
        symbols = unicode_symbols if console.has_unicode() else ascii_symbols
    tip_id = root.tip().id if root.tip() else None

    def get_behaviour_type(b):
        for behaviour_type in [composites.Sequence,
                               composites.Selector,
                               composites.Parallel,
                               decorators.Decorator]:
            if isinstance(b, behaviour_type):
                return behaviour_type
        return behaviour.Behaviour

    def style(s, font_weight=False):
        """
        Because the way the shell escape sequences reset everything, this needs to get used on any
        single block of formatted text.
        """
        if font_weight:
            return symbols['bold'] + s + symbols['bold_reset']
        else:
            return s

    def generate_lines(root, internal_indent):

        def assemble_single_line(b):
            font_weight = True if b.id == tip_id else False
            s = symbols['space'] * 4 * internal_indent
            s += style(symbols[get_behaviour_type(b)], font_weight)
            s += " "

            if show_status or b.id in visited.keys():
                s += style("{} [".format(b.name.replace('\n', ' ')), font_weight)
                s += style("{}".format(symbols[b.status]), font_weight)
                message = "" if not b.feedback_message else " -- " + b.feedback_message
                s += style("]" + message, font_weight)
            elif (b.id in previously_visited.keys() and
                  b.id not in visited.keys() and
                  previously_visited[b.id] == common.Status.RUNNING):
                s += style("{} [".format(b.name.replace('\n', ' ')), font_weight)
                s += style("{}".format(symbols[b.status]), font_weight)
                s += style("]", font_weight)
            else:
                s += style("{}".format(b.name.replace('\n', ' ')), font_weight)
            return s

        if internal_indent == indent:
            # Root
            yield assemble_single_line(root)
            internal_indent += 1
        for child in root.children:
            yield assemble_single_line(child)
            if child.children != []:
                for line in generate_lines(child, internal_indent + 1):
                    yield line
    s = ""
    for line in generate_lines(root, indent):
        s += "%s\n" % line
    return s


def ascii_tree(
        root,
        show_status=False,
        visited={},
        previously_visited={},
        indent=0,
     ):
    """
    Graffiti your console with ascii art for your trees.

    Args:
        root (:class:`~py_trees.behaviour.Behaviour`): the root of the tree, or subtree you want to show
        show_status (:obj:`bool`): always show status and feedback message (i.e. for every element, not just those visited)
        visited (dict): dictionary of (uuid.UUID) and status (:class:`~py_trees.common.Status`) pairs for behaviours visited on the current tick
        previously_visited (dict): dictionary of behaviour id/status pairs from the previous tree tick
        indent (:obj:`int`): the number of characters to indent the tree

    Returns:
        :obj:`str`: an ascii tree (i.e. in string form)

    .. seealso:: :meth:`py_trees.display.xhtml_tree`, :meth:`py_trees.display.unicode_tree`

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
                print(
                    py_trees.display.unicode_tree(
                        behaviour_tree.root,
                        visited=snapshot_visitor.visited,
                        previously_visited=snapshot_visitor.visited
                    )
                )

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
    lines = _generate_text_tree(
        root,
        show_status,
        visited,
        previously_visited,
        indent,
        symbols=ascii_symbols
    )
    return lines


def unicode_tree(
        root,
        show_status=False,
        visited={},
        previously_visited={},
        indent=0,
     ):
    """
    Graffiti your console with unicode art for your trees.

    Args:
        root (:class:`~py_trees.behaviour.Behaviour`): the root of the tree, or subtree you want to show
        show_status (:obj:`bool`): always show status and feedback message (i.e. for every element, not just those visited)
        visited (dict): dictionary of (uuid.UUID) and status (:class:`~py_trees.common.Status`) pairs for behaviours visited on the current tick
        previously_visited (dict): dictionary of behaviour id/status pairs from the previous tree tick
        indent (:obj:`int`): the number of characters to indent the tree

    Returns:
        :obj:`str`: a unicode tree (i.e. in string form)

    .. seealso:: :meth:`py_trees.display.ascii_tree`, :meth:`py_trees.display.xhtml_tree`

    """
    lines = _generate_text_tree(
        root,
        show_status,
        visited,
        previously_visited,
        indent,
        symbols=ascii_symbols
    )
    return lines


def xhtml_tree(
        root,
        show_status=False,
        visited={},
        previously_visited={},
        indent=0,
     ):
    """
    Paint your tree on an xhtml snippet.

    Args:
        root (:class:`~py_trees.behaviour.Behaviour`): the root of the tree, or subtree you want to show
        show_status (:obj:`bool`): always show status and feedback message (i.e. for every element, not just those visited)
        visited (dict): dictionary of (uuid.UUID) and status (:class:`~py_trees.common.Status`) pairs for behaviours visited on the current tick
        previously_visited (dict): dictionary of behaviour id/status pairs from the previous tree tick
        indent (:obj:`int`): the number of characters to indent the tree

    Returns:
        :obj:`str`: an ascii tree (i.e. as a xhtml snippet)

    .. seealso:: :meth:`py_trees.display.ascii_tree`, :meth:`py_trees.display.unicode_tree`

    Examples:

    .. code-block:: python

        import py_trees
        a = py_trees.behaviours.Success()
        b = py_trees.behaviours.Success()
        c = c = py_trees.composites.Sequence(children=[a, b])
        c.tick_once()

        f = open('testies.html', 'w')
        f.write('<html><head><title>Foo</title><body>')
        f.write(py_trees.display.xhtml_tree(c, show_status=True))
        f.write("</body></html>")
    """
    lines = _generate_text_tree(
        root,
        show_status,
        visited,
        previously_visited,
        indent,
        symbols=xhtml_symbols
    )
    lines = lines.replace("\n", "<br/>\n")
    return "<code>\n" + lines + "</code>"


def dot_tree(
        root: behaviour.Behaviour,
        visibility_level: common.VisibilityLevel=common.VisibilityLevel.DETAIL,
        collapse_decorators: bool=False,
        with_qualified_names: bool=False):
    """
    Paint your tree on a pydot graph.

    .. seealso:: :py:func:`render_dot_tree`.

    Args:
        root (:class:`~py_trees.behaviour.Behaviour`): the root of a tree, or subtree
        visibility_level (:class`~py_trees.common.VisibilityLevel`): collapse subtrees at or under this level
        collapse_decorators (:obj:`bool`, optional): only show the decorator (not the child), defaults to False
        with_qualified_names: (:obj:`bool`, optional): print the class information for each behaviour in each node, defaults to False

    Returns:
        pydot.Dot: graph

    Examples:

        .. code-block:: python

            # convert the pydot graph to a string object
            print("{}".format(py_trees.display.dot_graph(root).to_string()))
    """
    def get_node_attributes(node):
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

    def get_node_label(node_name, behaviour):
        """
        This extracts a more detailed string (when applicable) to append to
        that which will be used for the node name.
        """
        node_label = node_name
        if behaviour.verbose_info_string():
            node_label += "\n{}".format(behaviour.verbose_info_string())
        if with_qualified_names:
            node_label += "\n({})".format(utilities.get_fully_qualified_name(behaviour))
        return node_label

    fontsize = 9
    graph = pydot.Dot(graph_type='digraph')
    graph.set_name("pastafarianism")  # consider making this unique to the tree sometime, e.g. based on the root name
    # fonts: helvetica, times-bold, arial (times-roman is the default, but this helps some viewers, like kgraphviewer)
    graph.set_graph_defaults(fontname='times-roman')  # splines='curved' is buggy on 16.04, but would be nice to have
    graph.set_node_defaults(fontname='times-roman')
    graph.set_edge_defaults(fontname='times-roman')
    (node_shape, node_colour, node_font_colour) = get_node_attributes(root)
    node_root = pydot.Node(
        root.name,
        label=get_node_label(root.name, root),
        shape=node_shape,
        style="filled",
        fillcolor=node_colour,
        fontsize=fontsize,
        fontcolor=node_font_colour)
    graph.add_node(node_root)
    names = [root.name]

    def add_edges(root, root_dot_name, visibility_level, collapse_decorators):
        if isinstance(root, decorators.Decorator) and collapse_decorators:
            return
        if visibility_level < root.blackbox_level:
            for c in root.children:
                (node_shape, node_colour, node_font_colour) = get_node_attributes(c)
                node_name = c.name
                while node_name in names:
                    node_name += "*"
                names.append(node_name)
                # Node attributes can be found on page 5 of
                #    https://graphviz.gitlab.io/_pages/pdf/dot.1.pdf
                # Attributes that may be useful: tooltip, xlabel
                node = pydot.Node(
                    node_name,
                    label=get_node_label(node_name, c),
                    shape=node_shape,
                    style="filled",
                    fillcolor=node_colour,
                    fontsize=fontsize,
                    fontcolor=node_font_colour,
                )
                graph.add_node(node)
                edge = pydot.Edge(root_dot_name, node_name)
                graph.add_edge(edge)
                if c.children != []:
                    add_edges(c, node_name, visibility_level, collapse_decorators)

    add_edges(root, root.name, visibility_level, collapse_decorators)
    return graph


def render_dot_tree(root: behaviour.Behaviour,
                    visibility_level: common.VisibilityLevel=common.VisibilityLevel.DETAIL,
                    collapse_decorators: bool=False,
                    name: str=None,
                    target_directory: str=os.getcwd(),
                    with_qualified_names: bool=False):
    """
    Render the dot tree to .dot, .svg, .png. files in the current
    working directory. These will be named with the root behaviour name.

    Args:
        root (:class:`~py_trees.behaviour.Behaviour`): the root of a tree, or subtree
        visibility_level (:class`~py_trees.common.VisibilityLevel`): collapse subtrees at or under this level
        collapse_decorators (:obj:`bool`): only show the decorator (not the child)
        name (:obj:`str`): name to use for the created files (defaults to the root behaviour name)
        target_directory (:obj:`str`): default is to use the current working directory, set this to redirect elsewhere
        with_qualified_names (:obj:`bool`): print the class names of each behaviour in the dot node

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
    graph = dot_tree(root, visibility_level, collapse_decorators, with_qualified_names=with_qualified_names)
    filename_wo_extension_to_convert = root.name if name is None else name
    filename_wo_extension = utilities.get_valid_filename(filename_wo_extension_to_convert)
    filenames = {}
    for extension, writer in {"dot": graph.write, "png": graph.write_png, "svg": graph.write_svg}.items():
        filename = filename_wo_extension + '.' + extension
        pathname = os.path.join(target_directory, filename)
        print("Writing {}".format(pathname))
        writer(pathname)
        filenames[extension] = pathname
    return filenames

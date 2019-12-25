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
import typing
import uuid

from . import behaviour
from . import blackboard
from . import common
from . import composites
from . import console
from . import decorators
from . import utilities

##############################################################################
# Symbols
##############################################################################

unicode_symbols = {
    'space': ' ',
    'left_arrow': console.left_arrow,
    'right_arrow': console.right_arrow,
    'left_right_arrow': console.left_right_arrow,
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
    'left_arrow': '<-',
    'right_arrow': '->',
    'left_right_arrow': '<->',
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
    'left_arrow': '<text>&#x2190;</text>',
    'right_arrow': '<text>&#x2192;</text>',
    'left_right_arrow': '<text>&#x2194;</text>',
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

##############################################################################
# Trees
##############################################################################


def _generate_text_tree(
        root,
        show_status=False,
        visited={},
        previously_visited={},
        indent=0,
        symbols=None):
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
        indent=0):
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
        indent=0):
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
        indent=0):
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
        with_blackboard_variables: bool=False,
        with_qualified_names: bool=False):
    """
    Paint your tree on a pydot graph.

    .. seealso:: :py:func:`render_dot_tree`.

    Args:
        root (:class:`~py_trees.behaviour.Behaviour`): the root of a tree, or subtree
        visibility_level (optional): collapse subtrees at or under this level
        collapse_decorators (optional): only show the decorator (not the child), defaults to False
        with_blackboard_variables (optional): add nodes for the blackboard variables
        with_qualified_names (optional): print the class information for each behaviour in each node, defaults to False

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
        try:
            if node.blackbox_level != common.BlackBoxLevel.NOT_A_BLACKBOX:
                attributes = (attributes[0], 'gray20', blackbox_font_colours[node.blackbox_level])
        except AttributeError:
            # it's a blackboard client, not a behaviour, just pass
            pass
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
    blackboard_colour = "blue"  # "dimgray"
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
    behaviour_id_name_map = {root.id: root.name}

    def add_children_and_edges(root, root_dot_name, visibility_level, collapse_decorators):
        if isinstance(root, decorators.Decorator) and collapse_decorators:
            return
        if visibility_level < root.blackbox_level:
            if len(root.children) > 1:
                subgraph = pydot.Subgraph(
                    label="children_of_{}".format(root.name),
                    rank="same"
                )
            else:
                subgraph = None
            for c in root.children:
                (node_shape, node_colour, node_font_colour) = get_node_attributes(c)
                node_name = c.name
                while node_name in behaviour_id_name_map.values():
                    node_name += "*"
                behaviour_id_name_map[c.id] = node_name
                # Node attributes can be found on page 5 of
                #    https://graphviz.gitlab.io/_pages/pdf/dot.1.pdf
                # Attributes that may be useful: tooltip, xlabel
                node = pydot.Node(
                    name=node_name,
                    label=get_node_label(node_name, c),
                    shape=node_shape,
                    style="filled",
                    fillcolor=node_colour,
                    fontsize=fontsize,
                    fontcolor=node_font_colour,
                )
                if subgraph is not None:
                    subgraph.add_node(node)
                graph.add_node(node)
                edge = pydot.Edge(root_dot_name, node_name)
                graph.add_edge(edge)
                if c.children != []:
                    add_children_and_edges(c, node_name, visibility_level, collapse_decorators)
            if subgraph is not None:
                graph.add_subgraph(subgraph)

    add_children_and_edges(root, root.name, visibility_level, collapse_decorators)

    def create_blackboard_client_node(blackboard_client: blackboard.Blackboard):
        return pydot.Node(
            name=blackboard_client.name,
            label=blackboard_client.name,
            shape="ellipse",
            style="filled",
            color=blackboard_colour,
            fillcolor="gray",
            fontsize=fontsize - 2,
            fontcolor=blackboard_colour,
        )

    def add_blackboard_nodes(blackboard_id_name_map: typing.Dict[uuid.UUID, str]):
        data = blackboard.Blackboard.storage
        metadata = blackboard.Blackboard.metadata
        clients = blackboard.Blackboard.clients
        # add client (that are not behaviour) nodes
        for unique_identifier, client in clients.items():
            if unique_identifier not in blackboard_id_name_map:
                graph.add_node(
                    create_blackboard_client_node(client)
                )
        # add key nodes
        for key in blackboard.Blackboard.keys():
            try:
                value = utilities.truncate(str(data[key]), 20)
                label = key + ": " + "{}".format(value)
            except KeyError:
                label = key + ": " + "-"
            blackboard_node = pydot.Node(
                key,
                label=label,
                shape='box',
                style="filled",
                color=blackboard_colour,
                fillcolor='white',
                fontsize=fontsize - 1,
                fontcolor=blackboard_colour,
                width=0, height=0, fixedsize=False,  # only big enough to fit text
            )
            graph.add_node(blackboard_node)
            for unique_identifier in metadata[key].read:
                try:
                    edge = pydot.Edge(
                        blackboard_node,
                        blackboard_id_name_map[unique_identifier],
                        color=blackboard_colour,
                        constraint=False
                    )
                except KeyError:
                    edge = pydot.Edge(
                        blackboard_node,
                        clients[unique_identifier].__getattribute__("name"),
                        color=blackboard_colour,
                        constraint=False
                    )
                graph.add_edge(edge)
            for unique_identifier in metadata[key].write:
                try:
                    edge = pydot.Edge(
                        blackboard_id_name_map[unique_identifier],
                        blackboard_node,
                        color=blackboard_colour,
                        constraint=True
                    )
                except KeyError:
                    edge = pydot.Edge(
                        clients[unique_identifier].__getattribute__("name"),
                        blackboard_node,
                        color=blackboard_colour,
                        constraint=False
                    )
                graph.add_edge(edge)

    if with_blackboard_variables:
        blackboard_id_name_map = {}
        for b in root.iterate():
            for bb in b.blackboards:
                blackboard_id_name_map[bb.id()] = behaviour_id_name_map[b.id]
        add_blackboard_nodes(blackboard_id_name_map)

    return graph


def render_dot_tree(root: behaviour.Behaviour,
                    visibility_level: common.VisibilityLevel=common.VisibilityLevel.DETAIL,
                    collapse_decorators: bool=False,
                    name: str=None,
                    target_directory: str=os.getcwd(),
                    with_blackboard_variables: bool=False,
                    with_qualified_names: bool=False):
    """
    Render the dot tree to .dot, .svg, .png. files in the current
    working directory. These will be named with the root behaviour name.

    Args:
        root: the root of a tree, or subtree
        visibility_level: collapse subtrees at or under this level
        collapse_decorators: only show the decorator (not the child)
        name: name to use for the created files (defaults to the root behaviour name)
        target_directory: default is to use the current working directory, set this to redirect elsewhere
        with_blackboard_variables: add nodes for the blackboard variables
        with_qualified_names: print the class names of each behaviour in the dot node

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
    graph = dot_tree(
        root, visibility_level, collapse_decorators,
        with_blackboard_variables=with_blackboard_variables,
        with_qualified_names=with_qualified_names)
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

##############################################################################
# Blackboards
##############################################################################


def _generate_text_blackboard(
        key_filter: typing.Union[typing.Set[str], typing.List[str]]=None,
        regex_filter: str=None,
        client_filter: typing.Union[typing.Set[uuid.UUID], typing.List[uuid.UUID]]=None,
        keys_to_highlight: typing.List[str]=[],
        display_only_key_metadata: bool=False,
        indent: int=0,
        symbols: typing.Dict[str, str]=None) -> str:
    """
    Generate a text blackboard.

    Args:
        key_filter: filter on a set/list of blackboard keys
        regex_filter: filter on a python regex str
        client_filter: filter on a set/list of client uuids
        keys_to_highlight: list of keys to highlight
        display_only_key_metadata: (read/write access, ...) instead of values
        indent: the number of characters to indent the blackboard
        symbols: dictates formatting style
            (one of :data:`py_trees.display.unicode_symbols` || :data:`py_trees.display.ascii_symbols` || :data:`py_trees.display.xhtml_symbols`),
            defaults to unicode if stdout supports it, ascii otherwise

    Returns:
        a text-based representation of the behaviour tree

    .. seealso:: :meth:`py_trees.display.unicode_blackboard`
    """
    if symbols is None:
        symbols = unicode_symbols if console.has_unicode() else ascii_symbols

    def style(s, font_weight=False):
        if font_weight:
            return symbols['bold'] + s + symbols['bold_reset']
        else:
            return s

    def generate_lines(storage, metadata, indent):
        def assemble_value_line(key, value, apply_highlight, indent, key_width):
            s = ""
            lines = ('{0}'.format(value)).split('\n')
            if len(lines) > 1:
                s += console.cyan + indent + '{0: <{1}}'.format(key, key_width) + console.white + ":\n"
                for line in lines:
                    s += console.yellow + indent + "  {0}\n".format(line)
            else:
                s += console.cyan + indent + '{0: <{1}}'.format(key, key_width) + console.white + ": " + console.yellow + '{0}\n'.format(value) + console.reset
            return style(s, apply_highlight) + console.reset

        def assemble_metadata_line(key, metadata, apply_highlight, indent, key_width):
            s = ""
            s += console.cyan + indent + '{0: <{1}}'.format(key, key_width + 1) + ": "
            client_uuids = list(set(metadata.read) | set(metadata.write) | set(metadata.exclusive))
            prefix = ''
            metastrings = []
            for client_uuid in client_uuids:
                metastring = prefix + '{0}'.format(
                    utilities.truncate(
                        blackboard.Blackboard.clients[client_uuid].name, 11
                    )
                )
                metastring += ' ('
                if client_uuid in metadata.read:
                    metastring += 'r'
                if client_uuid in metadata.write:
                    metastring += 'w'
                if client_uuid in metadata.exclusive:
                    metastring += 'x'
                metastring += ')'
                metastrings.append(metastring)
            s += console.yellow + "{}\n".format(', '.join(metastrings))
            return style(s, apply_highlight) + console.reset

        text_indent = symbols['space'] * (4 + indent)
        key_width = 0
        for key in storage.keys():
            key_width = len(key) if len(key) > key_width else key_width
        for key in sorted(storage.keys()):
            if metadata is not None:
                yield assemble_metadata_line(
                    key=key,
                    metadata=metadata[key],
                    apply_highlight=key in keys_to_highlight,
                    indent=text_indent,
                    key_width=key_width)
            else:
                yield assemble_value_line(
                    key=key,
                    value=storage[key],
                    apply_highlight=key in keys_to_highlight,
                    indent=text_indent,
                    key_width=key_width)

    blackboard_metadata = blackboard.Blackboard.metadata if display_only_key_metadata else None

    if key_filter:
        if type(key_filter) == list:
            key_filter = set(key_filter)
        all_keys = blackboard.Blackboard.keys() & key_filter
    elif regex_filter:
        all_keys = blackboard.Blackboard.keys_filtered_by_regex(regex_filter)
    elif client_filter:
        all_keys = blackboard.Blackboard.keys_filtered_by_clients(client_filter)
    else:
        all_keys = blackboard.Blackboard.keys()
    blackboard_storage = {}
    for key in all_keys:
        try:
            blackboard_storage[key] = blackboard.Blackboard.storage[key]
        except KeyError:
            blackboard_storage[key] = "-"

    title = "Clients" if display_only_key_metadata else "Data"
    s = console.green + symbols['space'] * indent + "Blackboard {}\n".format(title) + console.reset
    if key_filter:
        s += symbols['space'] * (indent + 2) + "Filter: '{}'\n".format(key_filter)
    elif regex_filter:
        s += symbols['space'] * (indent + 2) + "Filter: '{}'\n".format(regex_filter)
    elif client_filter:
        s += symbols['space'] * (indent + 2) + "Filter: {}\n".format(str(client_filter))
    for line in generate_lines(blackboard_storage, blackboard_metadata, indent):
        s += "{}".format(line)
    return s


def ascii_blackboard(
        key_filter: typing.Union[typing.Set[str], typing.List[str]]=None,
        regex_filter: str=None,
        client_filter: typing.Union[typing.Set[uuid.UUID], typing.List[uuid.UUID]]=None,
        keys_to_highlight: typing.List[str]=[],
        display_only_key_metadata: bool=False,
        indent: int=0) -> str:
    """
    Graffiti your console with ascii art for your blackboard.

    Args:
        key_filter: filter on a set/list of blackboard keys
        regex_filter: filter on a python regex str
        client_filter: filter on a set/list of client uuids
        keys_to_highlight: list of keys to highlight
        display_only_key_metadata: read/write access, ... instead of values
        indent: the number of characters to indent the blackboard

    Returns:
        a unicoded blackboard (i.e. in string form)

    .. seealso:: :meth:`py_trees.display.unicode_blackboard`

    .. note:: registered variables that have not yet been set are marked with a '-'
    """
    lines = _generate_text_blackboard(
        key_filter=key_filter,
        regex_filter=regex_filter,
        client_filter=client_filter,
        keys_to_highlight=keys_to_highlight,
        display_only_key_metadata=display_only_key_metadata,
        indent=indent,
        symbols=ascii_symbols
    )
    return lines


def unicode_blackboard(
        key_filter: typing.Union[typing.Set[str], typing.List[str]]=None,
        regex_filter: str=None,
        client_filter: typing.Union[typing.Set[uuid.UUID], typing.List[uuid.UUID]]=None,
        keys_to_highlight: typing.List[str]=[],
        display_only_key_metadata: bool=False,
        indent: int=0) -> str:
    """
    Graffiti your console with unicode art for your blackboard.

    Args:
        key_filter: filter on a set/list of blackboard keys
        regex_filter: filter on a python regex str
        client_filter: filter on a set/list of client uuids
        keys_to_highlight: list of keys to highlight
        display_only_key_metadata: read/write access, ... instead of values
        indent: the number of characters to indent the blackboard

    Returns:
        a unicoded blackboard (i.e. in string form)

    .. seealso:: :meth:`py_trees.display.ascii_blackboard`

    .. note:: registered variables that have not yet been set are marked with a '-'
    """
    lines = _generate_text_blackboard(
        key_filter=key_filter,
        regex_filter=regex_filter,
        client_filter=client_filter,
        keys_to_highlight=keys_to_highlight,
        display_only_key_metadata=display_only_key_metadata,
        indent=indent,
        symbols=None  # defaults to unicode, falls back to ascii
    )
    return lines


def _generate_text_activity(
    activity_stream: typing.List[blackboard.ActivityItem]=None,
    show_title: bool=True,
    indent: int=0,
    symbols: typing.Dict[str, str]=None
) -> str:
    """
    Generator for the various formatted outputs (ascii, unicode, xhtml).

    Args:
        activity_stream: the log of activity, if None, get the entire activity stream
        indent: the number of characters to indent the blackboard
        show_title: include the title in the output
    """
    space = symbols['space']
    if activity_stream is None:
        activity_stream = blackboard.Blackboard.activity_stream.data
    if show_title:
        s = space * indent + console.green + "Blackboard Activity Stream" + console.reset + "\n"
    else:
        s = ""
    if activity_stream is not None:
        key_width = 0
        client_width = 0
        for item in activity_stream:
            key_width = len(item.key) if len(item.key) > key_width else key_width
            client_width = len(item.client_name) if len(item.client_name) > client_width else client_width
        client_width = min(client_width, 20)
        type_width = len("ACCESS_DENIED")
        value_width = 80 - key_width - 3 - type_width - 3 - client_width - 3
        for item in activity_stream:
            s += console.cyan + space * (4 + indent)
            s += "{0: <{1}}:".format(item.key, key_width + 1) + space
            s += console.yellow
            s += "{0: <{1}}".format(item.activity_type, type_width) + space
            s += console.white + "|" + space
            s += "{0: <{1}}".format(
                utilities.truncate(
                    item.client_name.replace('\n', '_'),
                    client_width),
                client_width) + space
            s += "|" + space
            if item.activity_type == blackboard.ActivityType.READ.value:
                s += symbols["left_arrow"] + space + "{}\n".format(
                    utilities.truncate(str(item.current_value), value_width)
                )
            elif item.activity_type == blackboard.ActivityType.WRITE.value:
                s += console.green
                s += symbols["right_arrow"] + space
                s += "{}\n".format(
                    utilities.truncate(str(item.current_value), value_width)
                )
            elif item.activity_type == blackboard.ActivityType.ACCESSED.value:
                s += console.yellow
                s += symbols["left_right_arrow"] + space
                s += "{}\n".format(
                    utilities.truncate(str(item.current_value), value_width)
                )
            elif item.activity_type == blackboard.ActivityType.ACCESS_DENIED.value:
                s += console.red
                s += console.multiplication_x + space
                s += "client has no read/write access\n"
            elif item.activity_type == blackboard.ActivityType.NO_KEY.value:
                s += console.red
                s += console.multiplication_x + space
                s += "key does not yet exist\n"
            elif item.activity_type == blackboard.ActivityType.NO_OVERWRITE.value:
                s += console.yellow
                s += console.forbidden_circle + space
                s += "{}\n".format(
                    utilities.truncate(str(item.current_value), value_width)
                )
            elif item.activity_type == blackboard.ActivityType.UNSET.value:
                s += "\n"
            elif item.activity_type == blackboard.ActivityType.INITIALISED.value:
                s += console.green
                s += symbols["right_arrow"] + space
                s += "{}\n".format(
                    utilities.truncate(str(item.current_value), value_width)
                )
            else:
                s += "unknown operation\n"
        s = s.rstrip("\n")
        s += console.reset
        return s


def unicode_blackboard_activity_stream(
    activity_stream: typing.List[blackboard.ActivityItem]=None,
    indent: int=0,
    show_title: bool=True
):
    """
    Pretty print the blackboard stream to console.

    Args:
        activity_stream: the log of activity, if None, get the entire activity stream
        indent: the number of characters to indent the blackboard
        show_title: include the title in the output
    """
    return _generate_text_activity(
        activity_stream=activity_stream,
        show_title=show_title,
        indent=indent,
        symbols=unicode_symbols if console.has_unicode() else ascii_symbols
    )

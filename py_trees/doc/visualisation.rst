Visualisation
=============

.. automodule:: py_trees.display
    :noindex:

Ascii Trees
-----------

You can get a very simple ascii representation of the tree on stdout with :func:`~py_trees.display.print_ascii_tree`:

.. autofunction:: py_trees.display.print_ascii_tree
    :noindex:

Ascii Trees (Runtime)
---------------------

When a tree is ticking, it is important to be able to catch the status and feedback message from each behaviour that
has been traversed. You can do this by using the :class:`~py_trees.visitors.SnapshotVisitor` in conjunction
with the :func:`~py_trees.display.ascii_tree` function:

.. autofunction:: py_trees.display.ascii_tree
    :noindex:

Dot Graphs
----------

You can render trees into dot/png/svg files simply by calling the :func:`~py_trees.display.render_dot_tree`
function.

.. autofunction:: py_trees.display.render_dot_tree
    :noindex:

They can also be converted to the dot graph format for export to other programs using the
:func:`~py_trees.display.generate_pydot_graph` method.
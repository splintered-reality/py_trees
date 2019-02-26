.. _visualisation-section:

Visualisation
=============

.. automodule:: py_trees.display
    :noindex:

.. _ascii-trees-section:

Ascii Trees
-----------

You can get a very simple ascii representation of the tree on stdout with :func:`~py_trees.display.print_ascii_tree`:

.. autofunction:: py_trees.display.print_ascii_tree
    :noindex:

.. _ascii-trees-runtime-section:

Ascii Trees (Runtime)
---------------------

When a tree is ticking, it is important to be able to catch the status and feedback message from each behaviour that
has been traversed. You can do this by using the :class:`~py_trees.visitors.SnapshotVisitor` in conjunction
with the :func:`~py_trees.display.ascii_tree` function:

.. autofunction:: py_trees.display.ascii_tree
    :noindex:

.. _render-to-file-section:

Render to File (Dot/SVG/PNG)
----------------------------

**API**

You can render trees into dot/png/svg files simply by calling the :func:`~py_trees.display.render_dot_tree`
function.

Should you wish to capture the dot graph result directly (as a dot graph object), use the
:func:`~py_trees.display.generate_pydot_graph` method.

**Command Line Utility**

You can also render any exposed method in your python packages that creates
a tree and returns the root of the tree from the command line using the :ref:`py-trees-render` program.

**Blackboxes and Visibility Levels**

There is also an experimental feature that allows you to flag behaviours as blackboxes with multiple levels
of granularity. This is purely for the purposes of showing different levels of detail in rendered dot graphs.
A fullly rendered dot graph with hundreds of behaviours is not of much use when wanting to visualise the
big picture.

The :ref:`py-trees-demo-dot-graphs-program` program serves as a self-contained example of this feature.

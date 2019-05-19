.. _visualisation-section:

Visualisation
=============

.. automodule:: py_trees.display
    :noindex:

.. _ascii-trees-section:

Ascii/Unicode Trees
-------------------

You can obtain an ascii/unicode art representation of the tree on stdout
via :func:`py_trees.display.ascii_tree` or :func:`py_trees.display.unicode_tree`:

.. autofunction:: py_trees.display.ascii_tree
    :noindex:

.. _render-to-file-section:

XHTML Trees
-----------

Similarly, :func:`py_trees.display.xhtml_tree` generates a static or runtime
representation of the tree as an embeddeble XHTML snippet. 

DOT Trees
---------

**API**

A static representation of the tree as a dot graph is obtained via
:func:`py_trees.display.dot_tree`. Should you wish to render the dot graph to
dot/png/svg images, make use of :meth:`py_trees.display.render_dot_tree`. Note that
the dot graph representation does not generate runtime information for the tree
(visited paths, status, ...).

**Command Line Utility**

You can also render any exposed method in your python packages that creates
a tree and returns the root of the tree from the command line using the
:ref:`py-trees-render` program. This is extremely useful when either designing your
trees or auto-rendering dot graphs for documentation on CI.

**Blackboxes and Visibility Levels**

There is also an experimental feature that allows you to flag behaviours as
blackboxes with multiple levels of granularity. This is purely for the
purposes of showing different levels of detail in rendered dot graphs.
A fullly rendered dot graph with hundreds of behaviours is not of much
use when wanting to visualise the big picture.

The :ref:`py-trees-demo-dot-graphs-program` program serves as a self-contained
example of this feature.

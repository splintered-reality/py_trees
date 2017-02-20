Visualisation
=============

Dot Graphs
----------

You can render trees into dot/png/svg files simply by calling the :py:func:`~py_trees.display.render_dot_tree`
function. There is also an ascii version.

.. code-block:: python

   root = py_trees.Sequence(name="Sequence")
   guard = py_trees.behaviours.Success("Guard")
   periodic_success = py_trees.behaviours.Periodic("Periodic", 3)
   finisher = py_trees.behaviours.Success("Finisher")
   root.add_child(guard)
   root.add_child(periodic_success)
   root.add_child(finisher)
   py_trees.display.render_dot_tree(root)

To enable quick generation of dotgraphs for your *subtrees*, use a class method inside your root class, e.g.

.. code-block:: python

   @classmethod
   def render_dot_tree(cls):
       root = cls()
       py_trees.display.render_dot_tree(root)

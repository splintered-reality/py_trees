.. _crazy-hospital-section:

Surviving the Crazy Hospital
============================

Your behaviour trees are misbehaving or your subtree designs seem overly
obtuse? This page can help you stay focused on what is important...staying out
of the padded room.

.. image:: images/crazy_hospital.jpg
    :width: 300px
    :align: center

.. note::
    Many of these guidelines we've evolved from trial and error and are almost
    entirely driven by a need to avoid a burgeoning complexity (aka
    :term:`flying spaghetti monster`). Feel free to experiment and provide us with
    your insights here as well!


Behaviours
----------

* Keep the constructor minimal so you can instantiate the behaviour for offline rendering
* Put hardware or other runtime specific initialisation in :meth:`~py_trees.behaviour.Behaviour.setup`
* The :meth:`~py_trees.behaviour.Behaviour.update` method must be light and non-blocking so a tree can keep ticking over
* Keep the scope of a single behaviour tight and focused, deploy larger reusable concepts as subtrees (idioms)

Composites
----------

* Avoid creating new composites, this increases the decision complexity by an order of magnitude
* Don't subclass merely to auto-populate it, build a :meth:`create_<xyz>_subtree` library instead

Trees
-----

* When designing your tree, stub them out with nonsense behaviours.
  Focus on descriptive names, composite types and render dot graphs
  to accelerate the design process (especially when collaborating). 
* Make sure your pre/post tick handlers and visitors are all very light.
* A good tick-tock rate for higher level decision making is around 1-500ms.

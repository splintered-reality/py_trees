The Crazy Hospital
==================

Your behaviour trees are misbehaving or your subtree designs seem overly
obtuse? This page can help you stay focused on what is important...staying out
of the padded room.

.. image:: images/crazy_hospital.jpg
    :width: 300px
    :align: center

Behaviours
----------

* Keep the constructor minimal so you can instantiate the behaviour for offline rendering
* Put hardware or other runtime specific initialisation in :meth:`~py_trees.behaviour.Behaviour.setup`
* Update :attr:`~py_trees.behaviour.Behaviour.feedback_message` for *significant events* only so you don't end up with too much noise
* The :meth:`~py_trees.behaviour.Behaviour.update` method must be light and non-blocking so a tree can keep ticking over

Composites
----------

* Avoid creating new composites, that path leads to the :term:`flying spaghetti monster`

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

* Keep the constructor minimal
* Hardware or other runtime specific initialisation in ``setup``
* Update feedback messages for **significant events** only
* A behaviour's ``update()`` method must be light and non-blocking


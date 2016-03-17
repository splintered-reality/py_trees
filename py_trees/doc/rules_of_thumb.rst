Rules of Thumb
==============

When to Initialise()?
---------------------

All behaviours/composites should call ``initialise()`` if not ``RUNNING`` upon entering ``tick()``, e.g.

.. code-block:: python

   def tick(self):
       if self.status != Status.RUNNING:
           self.initialise()

Switching Branches - Must Dos
-----------------------------

From one run to the next, if the currently running part of the tree gets switched out for a higher
priority part, this will always happen via a ``Selector``. When this occurs, the selector *will* always
send a ``stop(INVALID)`` to the previously running part of the tree.

This ``stop(INVALID)`` call *must* percolate down through all previously running composites and behaviours.

What Should go into Behaviour Messages?
---------------------------------------

*Only the significant things.*

This saves higher level programs (e.g. ``rqt_py_trees``) and hoomans be more tuned into
what's happening rather than having to filter the noise themselves. A good example of
this is the battery behaviour. You probably don't need to notify it changing every 1%.


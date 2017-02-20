Design Patterns
===============

Construct-Destruct
------------------

Behaviours have a natural construct-destruct mechanism in the
:func:`~py_trees.behaviour.Behaviour.setup`/:func:`~py_trees.behaviour.Behaviour.initialise`
and :func:`~py_trees.behaviour.Behaviour.terminate` functions. This however only allows you to
construct and destruct across the duration of that specific behaviour.

Quite often however, you would like to construct-destruct across a sequence of behaviours.
For example, you might (construct) switch a Navigation Context at the start of a sequence and then
(destruct) restore the Navigation Context at the end of that sequence.

A first attempt might try adding switching and restoring functionality in the
:func:`~py_trees.behaviour.Behaviour.initialise` function of the first and last behaviours. e.g.

.. graphviz:: dot/naive_context_switching.dot

However, what happens if a higher priority interrupts this sequnce while it is in the ``Move It``
behaviour? In this case the sequence will be cancelled and the restore context behaviour will never
be activated.

The solution is to utilise a :class:`~py_trees.composites.Parallel` and make use of the
:func:`~py_trees.behaviour.Behaviour.initialise`
and :func:`~py_trees.behaviour.Behaviour.terminate` functions of an appropriately created
context switching behaviour.

.. graphviz:: dot/context_switching.dot

Here the move behaviour will tick through to completion, at which point the parallel becomes
a success (:py:data:`~py_trees.common.ParallelPolicy.SUCCESS_ON_ONE`) and the context
behaviour is also promoted to a success via it's own terminate function where it will
restore the context. Example code with simplified class representations below.

.. literalinclude:: ./examples/context_switching.py
   :language: python
   :emphasize-lines: 23-26
   :linenos:

Cancelling
----------

Handling a **cancel on request** and a **cancel** on failure. The former is triggered by a
higher priority event, while the latter is triggered after a higher priority has failed.
This can be tricky to think about when you first attempt it, but it falls into a pattern
after a few attempts.

.. todo:: cancelling design pattern example

Timeout
-------

This is a variation of the cancelling design pattern.

.. todo:: timeout design pattern example


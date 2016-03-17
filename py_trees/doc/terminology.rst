Terminology
===========

.. glossary::

   tick
   ticks
   ticking
      When a **behaviour** ticks, it is executing a small, non-blocking chunk of code
      that checks a variable or triggers/monitors/returns the result of an external action.
      When a **behaviour tree** ticks, it traverses the behaviours (starting at the root of
      the tree), ticking each behaviour,
      catching its result and then using that result to make decisions on how to continue
      traversing the tree (go deeper, go the next leaf, go back up, ...). Behaviours with
      children (e.g. Sequences and Selectors) will delay their result until their
      children have confirmed a result for them. Once the traversal arrives back to the root
      of the tree, the tick is over.

      Once a tick is done..you can stop for breath! In this space you can pause to avoid
      eating the cpu, send some statistics out to a monitoring program, manipulate the
      underlying blackboard (data), have a coffee...This is absolutely awesome - without
      this it would be a concurrent mess of locks and threads.

      Concurrency for free...almost. Always, always keep in mind that your behaviours
      executions must be light. We are not parallelising here. In almost all cases, most of our
      robot behaviour trees are probably not going to be significantly expensive (unlike games
      with thousands of characters).

.. todo:: Add an image of a ticking tree here.


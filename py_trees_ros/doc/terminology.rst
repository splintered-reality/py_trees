Terminology
===========

.. glossary::

   tick
   ticks
   ticking
      A key feature of behaviours and their trees is in the way they *tick*. A tick
      is merely an execution slice, similar to calling a function once, or executing
      a loop in a control program once.

      When a **behaviour** ticks, it is executing a small, non-blocking chunk of code
      that checks a variable or triggers/monitors/returns the result of an external action.

      When a **behaviour tree** ticks, it traverses the behaviours (starting at the root of
      the tree), ticking each behaviour, catching its result and then using that result to
      make decisions on the direction the tree traversal will take. This is the decision part
      of the tree. Once the traversal ends back at the root, the tick is over.

      Once a tick is done..you can stop for breath! In this space you can pause to avoid
      eating the cpu, send some statistics out to a monitoring program, manipulate the
      underlying blackboard (data), ... At no point does the traversal of the tree get mired in
      execution - it's just in and out and then stop for a coffee. This is absolutely awesome
      - without this it would be a concurrent mess of locks and threads.

      Always keep in mind that your behaviours' executions must be light. There is no
      parallelising here. In almost all cases, most of your robot behaviour trees are probably
      not going to be significantly expensive (unlike games with thousands of characters).

      Add an image of a ticking tree here.

   blocking
      A behaviour is sometimes referred to as a 'blocking' behaviour. Technically, the execution
      of a behaviour should be non-blocking (i.e. the tick part), however when it's progress from
      'RUNNING' to 'FAILURE/SUCCESS' takes more than one tick, we say that the beahviour itself
      is blocking. In short, `blocking == RUNNING`.

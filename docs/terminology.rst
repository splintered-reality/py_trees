.. _terminology-section:

Terminology
===========

.. glossary::


   block
   blocking
      A behaviour is sometimes referred to as a 'blocking' behaviour. Technically, the execution
      of a behaviour should be non-blocking (i.e. the tick part), however when it's progress from
      'RUNNING' to 'FAILURE/SUCCESS' takes more than one tick, we say that the behaviour itself
      is blocking. In short, `blocking == RUNNING`.

   data gathering
      Caching events, notifications, or incoming data arriving asynchronously on the blackboard.
      This is a fairly common practice for behaviour trees which exist inside a complex system.

      In most cases, data gathering is done either outside the tree, or at the front end of your
      tree under a parallel preceding the rest of the tree tick so that the ensuing behaviours
      work on a constant, consistent set of data. Even if the incoming data is not arriving
      asynchronously, this is useful conceptually and organisationally.

   fsm
   flying spaghetti monster
      Whilst a serious religous entity in his own right (see `pastafarianism`_), it's also
      very easy to imagine your code become a spiritual flying spaghetti monster if left
      unchecked::

               _  _(o)_(o)_  _
             ._\`:_ F S M _:' \_,
                 / (`---'\ `-.
              ,-`  _)    (_,

   guard
      A guard is a behaviour at the start of a sequence that checks for a particular condition
      (e.g. is battery low?). If the check succeeds, then the door is opened to the rest of the
      work sequence.

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
      parallelising here and your tick time needs to remain small. The tree should be solely
      about decision making, not doing any actual blocking work. Any blocking work should be
      happening somewhere else with a behaviour simply in charge of starting/monitoring and
      catching the result of that work.

      Add an image of a ticking tree here.

.. _pastafarianism: http://www.venganza.org/

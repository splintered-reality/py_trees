Background
==========

Introduction
------------

Behaviour trees are a decision making engine often used in the gaming industry. Other engines
that are often used include hierarchical finite state machines, task networks, scripting
engines all of which have various pros and cons. Behaviour trees sit somewhere in the middle
of these allowing you a good blend of purposeful planning towards goals with enough reactivity
to shift in the presence of important events. They are also wonderfully simple to compose.

There's a wealth of information already covering behaviour trees. Rather than regurgitating
it in a lesser form here, dig through some of these first. A good starter is
`AI GameDev - Behaviour Trees`_ (free signup and login) which puts behaviour trees in context
alongside other techniques. Some other useful readings are listed at the bottom.

Some standout features of behaviour trees that makes them very attractive:

* **Ticking** - the ability to :term:`tick` allows us to work between executions without having to manage multi-threading
* **Priority Handling** - creating switching mechansims to allow interruption by higher priority tasks is very natural
* **Simplicity** - very few fundamental components, making it easy for designers and devs to work with it

Why Py Trees?
-------------

This is a python implementation that uses all the whizbang tricks (generators, decorators)
that python has to offer. It should be used for quickly scripting a decision making engine
of relatively finite size that don't need to be 'real time' reactive.

Handling the higher level planning and scenario decision making requirements in robotics is a good fit.
These systems typically do not grow too large (~ hundreds of behaviours) and do not need to handle
the reactive decision making (that is usually directly incorporated into the controller subsystems).

Do not try to use it for NPC's in a gaming engine which typically requires many tree instances
in parallel, with each tree covering tens of thousands of behaviours.

*(A parallel c++ version  of py_trees is feasible - once stable and there's a need...)*

What about other behaviour tree implementations? There are almost none that are open. Some have started, but
haven't progressed far (e.g. `Owyl`_). `Behaviour Designer`_ is a frontend to the trees in the unity framework.
Does this mean people do not use them? It is more likely that most behaviour tree
implementations happen within the closed doors of gaming/robot companies. `Youtube - Second Generation of Behaviour Trees`_
is an enlightening video about behaviour trees and the developments of the last ten years from an industry expert. He even
walks you through a simple c++ implementation. His advice? It is *relatively simple* to roll your own and given that the
scenery is still changing, this way you can more flexibly cater for your own needs.

Readings
--------

* `AI GameDev - Behaviour Trees`_ - from a gaming expert, good big picture view
* `Youtube - Second Generation of Behaviour Trees`_ - from a gaming expert, in depth c++ walkthrough (on github).
* `Behaviour trees for robotics`_ - by pirobot, a clear intro on its usefulness for robots.
* `A Curious Course on Coroutines and Concurrency`_ - generators and coroutines in python.

.. _Owyl: https://github.com/eykd/owyl
.. _AI GameDev - Behaviour Trees: http://aigamedev.com/insider/presentation/behavior-trees/
.. _Youtube - Second Generation of Behaviour Trees: https://www.youtube.com/watch?v=n4aREFb3SsU
.. _Behaviour trees for robotics: http://www.pirobot.org/blog/0030/
.. _A Curious Course on Coroutines and Concurrency: http://www.dabeaz.com/coroutines/Coroutines.pdf
.. _Behaviour Designer: https://forum.unity3d.com/threads/behavior-designer-behavior-trees-for-everyone.227497/


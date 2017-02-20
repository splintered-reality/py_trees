Background
==========

Introduction
------------

.. note:: Behaviour trees are a decision making engine often used in the gaming industry.

Other engines
that are often used include hierarchical finite state machines, task networks, scripting
engines all of which have various pros and cons. Behaviour trees sit somewhere in the middle
of these allowing you a good blend of purposeful planning towards goals with enough reactivity
to shift in the presence of important events. They are also wonderfully simple to compose.

There's a wealth of information already covering behaviour trees. Rather than regurgitating
it in a lesser form here, dig through some of these first. A good starter is
`AI GameDev - Behaviour Trees`_ (free signup and login) which puts behaviour trees in context
alongside other techniques. Some other useful readings are listed at the bottom.

Some standout features of behaviour trees that makes them very attractive:

* **Ticking** - the ability to :term:`tick` allows for work between executions without multi-threading
* **Priority Handling** - switching mechansims that allow higher priority interruptions is very natural
* **Simplicity** - very few core components, making it easy for designers to work with it

Motivation
----------

.. note:: Existing behaviour tree implementations? There are almost none that are open.

Some have started, but haven't progressed far (e.g. `Owyl`_).
`Behaviour Designer`_ is a frontend to the trees in the unity framework.
Does this mean people do not use them? It is more likely that most behaviour tree
implementations happen within the closed doors of gaming/robot companies. `Youtube - Second Generation of Behaviour Trees`_
is an enlightening video about behaviour trees and the developments of the last ten years from an industry expert. He even
walks you through a simple c++ implementation. His advice? Roll your own, it is relatively simple
this way you can flexibly cater for your own needs.

Design
------

.. note:: For rapid development of small scale decision engines that don't need to be real time reactive.

That is, don't try using it for an NPC gaming engine with
hundreds of characters and thousands of beahaviours for each character.
In contrast, handling the higher level planning and scenario decision making in robotics is a good fit.
These systems typically do not grow too large (~ hundreds of behaviours) and do not need to handle
the reactive decision making that is usually directly incorporated into the controller subsystems.

This implementation uses all the whizbang tricks (generators, decorators)
that python has to offer. Some design constraints that have been assumed to enable a practical, easy to use framework:

* No interaction or sharing of data between tree instances
* No parallelisation of tree execution
* Only one behaviour initialising or executing at a time

.. hint:: A c++ version is feasible and may come forth once py_trees is stable and there's a need...*


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


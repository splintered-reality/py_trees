Background
==========

.. _introduction-section:

Introduction
------------

.. note:: Behaviour trees are a decision making engine often used in the gaming industry.

Others include hierarchical finite state machines, task networks, and scripting
engines, all of which have various pros and cons. Behaviour trees sit somewhere in the middle
of these allowing you a good blend of purposeful planning towards goals with enough reactivity
to shift in the presence of important events. They are also wonderfully simple to compose.

There's much information already covering behaviour trees. Rather than regurgitating
it here, dig through some of these first. A good starter is
`AI GameDev - Behaviour Trees`_ (free signup and login) which puts behaviour trees in context
alongside other techniques. A simpler read is Patrick Goebel's `Behaviour Trees For Robotics`_.
Other readings are listed at the bottom of this page.

Some standout features of behaviour trees that makes them very attractive:

* **Ticking** - the ability to :term:`tick` allows for work between executions without multi-threading
* **Priority Handling** - switching mechansims that allow higher priority interruptions is very natural
* **Simplicity** - very few core components, making it easy for designers to work with it
* **Dynamic** - change the graph on the fly, between ticks or from parent behaviours themselves

.. _motivation-section:

Motivation
----------

The driving use case for this package was to implement a higher level decision making layer in robotics, i.e.
scenarios with some overlap into the control layer. Behaviour trees turned out to be a much more
apt fit to handle the many concurrent processes in a robot after attempts with finite state machines
became entangled in wiring complexity as the problem grew in scope.

.. note:: There are very few open behaviour tree implementations.

Most of these have either not progressed significantly (e.g. `Owyl`_), or are
accessible only in some niche, e.g. `Behaviour Designer`_, which is a frontend to the trees in the unity framework.
Does this mean people do not use them? It is more probable that most behaviour tree
implementations happen within the closed doors of gaming/robot companies.

`Youtube - Second Generation of Behaviour Trees`_ is an enlightening video about behaviour trees and
the developments of the last ten years from an industry expert. It also
walks you through a simple c++ implementation. His advice? If you can't find one that fits, roll your own.
It is relatively simple and this way you can flexibly cater for your own needs.

.. _design-section:

Design
------

The requirements for the previously discussed robotics use case match that of the more general:

.. note:: Rapid development of medium scale decision engines that don't need to be real time reactive.

Developers should expect to be able to get up to speed and write their own trees with enough
power and flexibility to adapt the library to their needs. Robotics is a good fit.
The decision making layer typically does not grow too large (~ hundreds of behaviours) and does not
need to handle the reactive decision making that is usually directly incorporated into the controller subsystems.
On the other hand, it is not scoped to enable an NPC gaming engine with hundreds of characters and thousands
of behaviours for each character.

This implementation uses all the whizbang tricks (generators, decorators)
that python has to offer. Some design constraints that have been assumed to enable a practical, easy to use framework:

* No interaction or sharing of data between tree instances
* No parallelisation of tree execution
* Only one behaviour initialising or executing at a time

.. hint:: A c++ version is feasible and may come forth if there's a need..

.. _readings-section:

Readings
--------

* `AI GameDev - Behaviour Trees`_ - from a gaming expert, good big picture view
* `Youtube - Second Generation of Behaviour Trees`_ - from a gaming expert, in depth c++ walkthrough (on github).
* `Behaviour trees for robotics`_ - by pirobot, a clear intro on its usefulness for robots.
* `A Curious Course on Coroutines and Concurrency`_ - generators and coroutines in python.
* `Behaviour Trees in Robotics and AI`_ - a rather verbose, but chock full with examples and comparisons with other approaches.

.. _Owyl: https://github.com/eykd/owyl
.. _AI GameDev - Behaviour Trees: http://aigamedev.com/insider/presentation/behavior-trees/
.. _Youtube - Second Generation of Behaviour Trees: https://www.youtube.com/watch?v=n4aREFb3SsU
.. _Behaviour Trees For Robotics: http://www.pirobot.org/blog/0030/
.. _A Curious Course on Coroutines and Concurrency: http://www.dabeaz.com/coroutines/Coroutines.pdf
.. _Behaviour Designer: https://forum.unity3d.com/threads/behavior-designer-behavior-trees-for-everyone.227497/
.. _Behaviour Trees in Robotics and AI: https://arxiv.org/pdf/1709.00084.pdf


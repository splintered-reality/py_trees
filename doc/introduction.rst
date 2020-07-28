Introduction
============

Quick Start
-----------

If you'd like to fast forward to some action, browse the :ref:`demos-section-label` or
read through the `ROS2 Robotics Tutorials`_ which incrementally create a significantly
more complex behaviour tree for a robotics scenario (ROS2 knowledge not needed).

.. _`ROS2 Robotics Tutorials`: https://py-trees-ros-tutorials.readthedocs.io/en/release-2.0.x/tutorials.html

.. _background-section:

Background
----------

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

.. note:: There are very few open behaviour tree implementations.

Most of these have either not progressed significantly (e.g. `Owyl`_), or are
accessible only in some niche, e.g. `Behaviour Designer`_, which is a frontend to the trees in the unity framework.
Does this mean people do not use them? It is more probable that most behaviour tree
implementations happen within the closed doors of gaming/robot companies.

`Youtube - Second Generation of Behaviour Trees`_ is an enlightening video about behaviour trees and
the developments of the last ten years from an industry expert. It also
walks you through a simple c++ implementation. His advice? If you can't find one that fits, roll your own.
It is relatively simple and this way you can flexibly cater for your own needs.

.. _motivation-section:

Motivation
----------

The use case that drove the early development of py_trees was robotics. In particular, the higher level
decision making for a single robot, i.e. the scenario layer. For example, the scenario that enables a
robot to navigate through a building to deliver a parcel and return to it's homebase safely.

In scope was any decision making that did not need a low-latency response (typically reactive safety
control measures). This included docking/undocking processes, the initial localisation dance,
topological path planning, navigation context switching, LED and sound interactions, elevator
entry/exit decisions.

Also driving requirements was the need to offload scenario development to non-control engineers
(juniors, interns, SWE's) and ensure they could develop and debug as rapidly as possible.

Behaviour trees turned out to be a perfect fit after attempts with finite state machines
became entangled in wiring complexity as the problem grew in scope.

.. _design-section:

Design
------

The requirements for the previously discussed robotics use case match that of the more general:

.. note:: **Rapid development** of **medium scale** decision engines that do **not need to be real time reactive**.

**Rapid Development**: Python was chosen as the language of choice since it enables a faster a cycle of development as
well as a shorter learning curve (critical if you would like to shift the burden away from c++ control engineers
to juniors/interns/software engineers).

**Medium Scale**: Robotic scenarios for a single robot tend to be, maximally in the order of hundreds of behaviours. This is
in contrast to game NPC's which need to be executing thousands of behaviours and/or trees and consequently, frequently
run into problems of scale. This tends to influence the language of choice (c++) and the tree design. Our requirements
are somewhat more modest, so this permits some flexibility in the design, e.g. python as a language of choice.

**Not Real Time Reactive**: If low latency control measures, particularly for safety are needed, they are best handled
directly inside the control layer, or even better, at an embedded level. This is not dissimilar to the way the
human nervous system operates. All other decision making needs only to operate at a latency of ~50-200ms
to negate any discernable delay observed by humans interacting with the robot.

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


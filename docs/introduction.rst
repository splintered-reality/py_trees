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

.. note::

   Behaviour trees are a decision making engine often used in the gaming and robotics industries.

Other decision making engines include hierarchical finite state machines, task networks, and scripting
engines, all of which have various pros and cons. Behaviour trees sit somewhere in the middle
of these allowing a bend of purposeful planning towards goals with enough reactivity
to shift in the presence of important events. Some standout features:

* **Ticking** - the ability to :term:`tick` allows for work between executions without multi-threading
* **Priority Handling** - switching mechansims that allow higher priority interruptions is very natural
* **Simplicity** - very few core components, making it easy for designers to work with it
* **Scalable** - do not suffer from combinatorial explosion as nodes increase (as state machines do)
* **Dynamic** - change the graph on the fly, between ticks or from parent behaviours themselves

In some texts, 'priority handling' is often referred to as 'reactivity'. There's much
information already covering behaviour trees, in particulary you may like to get started with:  

* `Introduction to Behavior Trees`_ - a gentle, practical introduction (2021)
* `Simon Jones' Thesis`_ (Ch3) - a computer scientist's attention to detail, (2020)

Other readings are listed at the bottom of this page.

.. _motivation-section:

Motivation
----------

The use case that drove the early development of py_trees was robotics. In particular, the higher level
decision making for a single robot, i.e. the scenario / application layer. For example, the scenario
that enables a robot to navigate through a building to deliver a parcel and return to it's
homebase safely.

In scope was any decision making that did not need a low-latency response (e.g. reactive safety
controls). This included docking/undocking processes, the initial localisation dance,
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

.. hint::

    If you wish to handle this requirement, you're likely looking for a c++ implementation with mechanisms
    for on-demand (not periodic) triggering of ticks to keep latency as minimal as can be.

**Python**: This implementation uses all the whizbang tricks (generators, decorators) that python has to offer.
Some design constraints that have been assumed to enable a practical, easy to use (and lock-free) framework:

* No interaction or sharing of data between tree instances
* No parallelisation of tree execution (only one behaviour initialising or executing at a time)

.. _readings-section:

Readings
--------

* `Introduction to Behavior Trees`_ - a gentle, practical introduction (2021)
* `Simon Jones' Thesis`_ - a computer scientist's treatise, nice attention to detail w.r.t. tree algorithms (2020, Ch.3)
* `Behaviour Trees in Robotics and AI`_ - a complete text, many examples and comparisons with other approaches.
* `Youtube - Second Generation of Behaviour Trees`_ - from a gaming expert, in depth c++ walkthrough (on github).
* `A Curious Course on Coroutines and Concurrency`_ - generators and coroutines in python.

Alternatives
------------

* `BehaviorTreeCPP`_ - a c++ open source implementation
* `UE4 Behavior Trees`_ - a closed implementation for use with unreal engine 

.. _UE4 Behavior Trees: https://docs.unrealengine.com/4.26/en-US/InteractiveExperiences/ArtificialIntelligence/BehaviorTrees/
.. _BehaviorTreeCPP: https://github.com/BehaviorTree/BehaviorTree.CPP
.. _Owyl: https://github.com/eykd/owyl
.. _Youtube - Second Generation of Behaviour Trees: https://www.youtube.com/watch?v=n4aREFb3SsU
.. _Introduction to Behavior Trees: https://roboticseabass.com/2021/05/08/introduction-to-behavior-trees/
.. _Behaviour Trees in Robotics and AI: https://btirai.github.io/index
.. _A Curious Course on Coroutines and Concurrency: http://www.dabeaz.com/coroutines/Coroutines.pdf
.. _Behaviour Designer: https://forum.unity3d.com/threads/behavior-designer-behavior-trees-for-everyone.227497/
.. _Simon Jones' Thesis: https://research-information.bris.ac.uk/ws/portalfiles/portal/225580708/simon_jones_thesis_final_accepted.pdf

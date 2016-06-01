Background
==========

Introduction
------------

Patrick Goebel wrote an introduction on his block about `Behaviour trees for robotics`_.
Rather than regurgitating alot of good information, go read that first. To follow that
up, sign up (its free) and watch `AI GameDev - Behaviour Trees`_ which is a much more
thorough analysis of behaviour trees from a gaming expert. It also includes a great
big picture overview of behaviour trees and where it fits in alongside other techniques.

Why Py Trees?
-------------

We needed a way to model the overall behaviour of a robot and didn't find anything that really fit till
we tried behaviour trees. For us a few standout points include:

* **Ticking** - the ability to :term:`tick` allows us to work between executions without having to manage multi-threading
* **Higher Priority Handling** - interrupting the current state is dead simple, no complicated wiring needed
* **Simplicity** - there are very few fundamental components, making it easy for designers and devs to work with it

It's worth noting that we did try smach, but it couldn't scale up to handle the problems we were trying
to solve.

This is a python implementation that uses all the whizbang tricks (generators, decorators)
that python has to offer. It doesn't have the speed of c++, however most of our robot scenarios do not need an
implementation that can handle thousands of behaviours at once. If we do, then we'll make that c++ implementation :)

Other behaviour tree implementations? There are almost none that are open. Some have started, but
haven't progressed far (e.g. `Owyl`_). Does this mean people don't use them? It is more likely that most behaviour tree
implementations happen within the closed doors of gaming/robot companies. `Youtube - Second Generation of Behaviour Trees`_
is an enlightening video about behaviour trees and the developments of the last ten years from an industry expert. He even
walks you through a simple c++ implementation. His advice? It is *relatively simple* to roll your own and given that the
scenery is still changing, this way you can more flexibly cater for your own needs....which in our case is for robotics.

Readings
--------

* `AI GameDev - Behaviour Trees`_ - by another gaming expert, good big picture view on behaviour trees.
* `Youtube - Second Generation of Behaviour Trees`_ - by a gaming expert, in depth c++ tutorial (on github).
* `Behaviour trees for robotics`_ - by pirobot, a clear intro on its usefulness for robots.
* `A Curious Course on Coroutines and Concurrency`_ - highlighting generators and coroutines in python.

.. _Owyl: https://github.com/eykd/owyl
.. _AI GameDev - Behaviour Trees: http://aigamedev.com/insider/presentation/behavior-trees/
.. _Youtube - Second Generation of Behaviour Trees: https://www.youtube.com/watch?v=n4aREFb3SsU
.. _Behaviour trees for robotics: http://www.pirobot.org/blog/0030/
.. _A Curious Course on Coroutines and Concurrency: http://www.dabeaz.com/coroutines/Coroutines.pdf


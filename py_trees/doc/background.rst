Background
==========

Introduction
------------

Patrick Goebel wrote a very good introduction on his block about `Behaviour trees for robotics`_.
Rather than regurgitating alot of good information, go read that first to get a good clue about
behaviour trees for robotics.

The Simplicity of Three
-----------------------

While it is possible to arbitrarily compose/extend any kind of logical building block for a behaviour tree,
it is surprising just how much can be done with just three - the **Behaviour**, **Selector** and **Sequence**.

Behaviours are a single entity, whilst selectors and sequences are compositions of behaviours. This is
amazingly simple, and once you get the hang of it, flexible to the point that its hard to think of anything
outside the three that you'd ever need. That is also a powerful indicator that it is the right tool (compare
with the exponentially growing headaches of increasingly large state machines).

Ticking
-------

More about this later, but a key feature of behaviour trees is the way they 'tick'. When you tick, you quickly
traverse a behaviour tree, making checks and decisions and initiating actions along the way. The emphasis is on being
quick. At no point does the traversal of the tree get stuck - it's just in and out and then stop for a coffee. Let
everyone else do the real work.

This design principle on its own brings very useful advantages. No need for locking/threads. You can analyse the data
from a tree between ticks at your leisure (extremely useful for monitoring). This all results in much less programming
and programming complexity and lets you concentrate on the decision logic alone.

Trees vs State Machines
-----------------------

Priority handling and ticking (or avoiding locking/threading). Enabling either of these in state machines is a
nightmare for anything sufficiently complex. The best example is to consider a large state machine for a robot
that has one necessary check - is my battery low, and one action - go to homebase and dock. In addition, this
must be checked no matter what state you are in. At this point you have to start thinking about concurrency,
or doing an incredibly large amount of wiring just to handle this one extra addition to your state machine.

*Tiny features should require a proportionally tiny amount of effort to integrate them.*

There is still a place for state machines, especially where priority handling is not important.

Why Py Trees?
-------------

We needed a way to model the overall behaviour of a robot and didn't find anything that really fit the bill till
we tried behaviour trees and were pleasantly surprised with the results.

This is a python implementation that uses all the whizbang tricks (generators, decorators) that python has to offer
to make it as fast as possible in python. Nonetheless, it's not a c++ implementation of behaviour trees - however
robot scenarios are more than often not likely to need a full blown c++ implementation that would handle thousands
of behaviours at once. If we do, then we'll make a c++ implementation :)

What about other behaviour tree implementations? Actually there are almost none that are open. Some have started, but
haven't progressed far (e.g. `Owyl`_). Does this mean people don't use them? It is more likely that most behaviour tree
implementations happen within the closed doors of gaming companies. `Youtube - Second Generation of Behaviour Trees`_
is an enlightening video about behaviour trees and the developments of the last ten years from an industry expert. He even
walks you through a simple c++ implementation. His advice? Go it on your own - they are *relatively simple* to build
and your own implementation can cater more flexibly for your own needs.

*Hence this implementation for our needs in robotics....*

Readings
--------

These aren't sorted - it is a mix of readings for both users and developers.

* `Youtube - Second Generation of Behaviour Trees`_ - in depth tutorial with game programming expert (on github).
* `Behaviour trees for robotics`_ - blogged by pirobot, a very clear introduction on its usefulness for robots.
* `A Curious Course on Coroutines and Concurrency`_ - very good presentation highlighting generators and coroutines in python.

.. _Owyl: https://github.com/eykd/owyl
.. _Youtube - Second Generation of Behaviour Trees: https://www.youtube.com/watch?v=n4aREFb3SsU
.. _Behaviour trees for robotics: http://www.pirobot.org/blog/0030/
.. _A Curious Course on Coroutines and Concurrency: http://www.dabeaz.com/coroutines/Coroutines.pdf


Background
==========

Introduction
------------

Patrick Goebel wrote a very good introduction on his block about `Behaviour trees for robotics`_.
Rather than regurgitating alot of good information, go read that first.

The Simplicity of Three
-----------------------

While it is possible to arbitrarily compose/extend any kind of logical building block for a behaviour tree,
it is surprising just how much can be done with just three - the **Behaviour**, **Selector** and **Sequence**.
Behaviours are a single entity, whilst selectors and sequences are compositions of behaviours. This trio is
simple, yet able to deliver surprisingly rich results.

Ticking
-------

A key feature of behaviour trees is the way they 'tick'. To tick a behaviour is to execute some small chunk of
code on that behaviour to arrive at a decision - success, failure, or still pending (running). To tick an entire
tree is to start ticking behaviours from the root, using their decisions and the selector-sequence
logic to choose which behaviour should next be ticked. This results in traversing the tree (though not necessarily
all of it) at the end of which the root of the tree will return a result.

The emphasis is on any execution being light and quick. At no point does the traversal of the tree get mired in
execution - it's just in and out and then stop for a coffee. Let everyone else do the real work. Typical behaviours
tend to monitor and/or trigger an external process, or do a quick status check.

Ticking over a tree brings very useful advantages. No need for locking/threads. You can analyse the data
from a tree between ticks at your leisure. This results in much less programming
and programming complexity and lets you concentrate on the decision logic alone.

Trees vs State Machines
-----------------------

The main differences lie in *priority handling* and *ticking* (or avoiding locking/threading).
Enabling either of these in state machines is a nightmare for anything sufficiently complicated.
The best example is to consider a large state machine for a robot
that has one necessary check - is my battery low?, and one action - go to homebase and recharge. In addition, this
must be checked no matter what state you are in. At this point you have to start thinking about concurrency,
or doing an incredibly large amount of wiring just to handle this one extra addition to your state machine.

*Small features should require a proportionally small amount of effort to integrate them.*

Why Py Trees?
-------------

We needed a way to model the overall behaviour of a robot and didn't find anything that really fit till
we tried behaviour trees. This is a python implementation that uses all the whizbang tricks (generators, decorators)
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

* `Youtube - Second Generation of Behaviour Trees`_ - in depth tutorial with game programming expert (on github).
* `Behaviour trees for robotics`_ - blogged by pirobot, a very clear introduction on its usefulness for robots.
* `A Curious Course on Coroutines and Concurrency`_ - very good presentation highlighting generators and coroutines in python.

.. _Owyl: https://github.com/eykd/owyl
.. _Youtube - Second Generation of Behaviour Trees: https://www.youtube.com/watch?v=n4aREFb3SsU
.. _Behaviour trees for robotics: http://www.pirobot.org/blog/0030/
.. _A Curious Course on Coroutines and Concurrency: http://www.dabeaz.com/coroutines/Coroutines.pdf


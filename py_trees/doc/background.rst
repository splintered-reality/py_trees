Background
==========

Resources
---------

* `Youtube - Second Generation of Behaviour Trees`_ - in depth tutorial with game programming expert (on github).
* `Behaviour trees for robotics`_ - blogged by pirobot, a very clear introduction on its usefulness for robots.
* `A Curious Course on Coroutines and Concurrency`_ - very good presentation highlighting generators and coroutines in python.

.. _Youtube - Second Generation of Behaviour Trees: https://www.youtube.com/watch?v=n4aREFb3SsU
.. _Behaviour trees for robotics: http://www.pirobot.org/blog/0030/
.. _A Curious Course on Coroutines and Concurrency: http://www.dabeaz.com/coroutines/Coroutines.pdf

In general, store, pass around and manipulate rocon uri's as strings. Create
:class:`BehaviourTree <py_trees.trees.BehaviourTree>` objects only as needed when you need to identify
individual elements of the uri or or you need to do comparisons.
This is a convention generally followed by urllib as well and is easiest in practice.


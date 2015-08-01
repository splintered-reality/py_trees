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

Parsing w/ Python Module
------------------------

To create/validate a rocon uri object, simply pass in the rocon uri string(s) to the parser and
access via the specialised descriptors:

.. code-block:: python

   try:
       rocon_uri_object = rocon_uri.parse('rocon:/turtlebot2|pr2/dude/hydro/precise#rocon_apps/chirp')
       print rocon_uri_object.hardware_platform.list       # output: ['turtlebot2', 'pr2']
       print rocon_uri_object.hardware_platform.string     # output: 'turtlebot2|pr2'
       print rocon_uri_object.name.string                  # output: 'dude'
       print rocon_uri_object.application_framework.string # output: 'hydro'
       print rocon_uri_object.operating_system.string      # output: 'precise'
       print rocon_uri_object.rapp                         # output: 'rocon_apps/chirp'
   except rocon_uri.RoconURIValueError as e:
       print("Invalid rocon uri string [%s]" % str(e))

Note the calls to the hardware_platform field via the list and string accessors. This is the same for
any of the primary fields stored by a :class:`RoconURI <rocon_uri.uri.RoconURI>` object

* hardware_platform
* name
* application_framework
* operating_system

The fragment part (rapp) does not follow the same rule and can be more simply accessed directly as shown.

Compatibility Testing w/ Python Module
--------------------------------------

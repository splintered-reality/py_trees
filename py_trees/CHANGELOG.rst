Changelog
=========

Forthcoming
-----------
* [ros] components moved to py_trees_ros
* [timeout] bugfix to ensure timeout decorator initialises properly
* [docs] rolled over with napolean style
* [docs] sphinx documentation updated
* [imposter] make sure tip() drills down into composites
* [demos] re-organised into modules

0.4.0 (2017-01-13)
------------------
* [trees] add pre/post handlers after setup, just in case setup fails
* [introspection] do parent lookups so you can crawl back up a tree
* [blackboard] permit init of subscriber2blackboard behaviours
* [blackboard] watchers
* [timers] better feedback messages
* [imposter] ensure stop() directly calls the composited behaviour

0.3.0 (2016-08-25)
------------------
* ``failure_is_running decorator`` (meta).

0.2.0 (2016-06-01)
------------------
* do terminate properly amongst relevant classes
* blackboxes
* chooser variant of selectors
* bugfix the decorators
* blackboard updates on change only
* improved dot graph creation
* many bugfixes to composites
* subscriber behaviours
* timer behaviours

0.1.2 (2015-11-16)
------------------
* one shot sequences
* abort() renamed more appropriately to stop()

0.1.1 (2015-10-10)
------------------
* lots of bugfixing stabilising py_trees for the spain field test
* complement decorator for behaviours
* dot tree views
* ascii tree and tick views
* use generators and visitors to more efficiently walk/introspect trees
* a first implementation of behaviour trees in python

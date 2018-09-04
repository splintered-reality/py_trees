Changelog
=========

Forthcoming
-----------

0.6.3 (2018-09-04)
------------------
* Parallels bugfix - don't send own status to running children, invalidate them instead

0.6.2 (2018-08-31)
------------------
* Oneshot bugfix - react to priority interrupts correctly

0.6.1 (2018-08-20)
------------------
* Oneshot bugfix - no longer permanently modifies the original class

0.6.0 (2018-05-15)
------------------
* Python 2/3 compatibility

0.5.10 (2017-06-17)
-------------------
* [meta] add children monkeypatching for composite imposters
* [blackboard] check for nested variables in WaitForBlackboard

0.5.9 (2017-03-25)
------------------
* [docs] bugfix image links and rewrite the motivation

0.5.8 (2017-03-19)
------------------
* [infra] setup.py tests_require, not test_require

0.5.7 (2017-03-01)
------------------
* [infra] update maintainer email

0.5.5 (2017-03-01)
------------------
* [docs] many minor doc updates
* [meta] bugfix so that imposter now ticks over composite children
* [trees] method for getting the tip of the tree
* [programs] py-trees-render program added

0.5.4 (2017-02-22)
------------------
* [infra] handle pypi/catkin conflicts with install_requires

0.5.2 (2017-02-22)
------------------
* [docs] disable colour when building
* [docs] sidebar headings
* [docs] dont require project installation

0.5.1 (2017-02-21)
------------------
* [infra] pypi package enabled

0.5.0 (2017-02-21)
------------------
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

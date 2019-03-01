Release Notes
=============

1.0.5 (2019-02-28)
------------------
* [decorators] fix timeout bug that doesn't respect a child's last tick

1.0.4 (2019-02-26)
------------------
* [display] drop spline curves, it's buggy with graphviz 2.38

1.0.3 (2019-02-13)
------------------
* [visitors] winds of change visitor and logging demo

1.0.2 (2019-02-13)
------------------
* [console] fallbacks for unicode chars when (UTF-8) encoding cannot support them

1.0.1 (2018-02-12)
------------------
* [trees] don't use multiprocess on setup if not using timeouts

1.0.x (2019-01-18)
------------------

**Breaking API**

* [behaviour] setup() no longer returns a boolean, catch exceptions instead, `#143 <https://github.com/stonier/py_trees/issues/143>`_.
* [behaviour] setup() no longer takes timeouts, responsibility moved to BehaviourTree, `#148 <https://github.com/stonier/py_trees/issues/148>`_.
* [decorators] new-style decorators found in py_trees.decorators
* [decorators] new-style decorators stop their running child on completion (SUCCESS||FAILURE)
* [decorators] old-style decorators in py_trees.meta deprecated

**New Features**

* [blackboard] added a method for clearing the entire blackboard (useful for tests)
* [composites] raise TypeError when children's setup methods don't return a bool (common mistake)
* [composites] new parallel policies, SuccessOnAll, SuccessOnSelected
* [decorators] oneshot policies for activating on completion or *successful* completion only
* [meta] behaviours from functions can now automagically generate names

0.8.x (2018-10-18)
------------------

**Breaking API**

* Lower level namespace types no longer exist (PR117_), e.g. :code:`py_trees.Status` -> :code:`py_trees.common.Status`
* Python2 support dropped

**New Features**

* [idioms] 'Pick Up Where You Left Off'
* [idioms] 'OneShot'

0.8.0 (2018-10-18)
------------------
* [infra] shortcuts to types in __init__.py removed (PR117_)
* [bugfix] python3 rosdeps
* [idioms] pick_up_where_you_left_off added

0.7.5 (2018-10-10)
------------------
* [idioms] oneshot added
* [bugfix] properly set/reset parents when replacing/removing children in composites

0.7.0 (2018-09-27)
------------------
* [announce] python3 only support from this point forward
* [announce] now compatible for ros2 projects

0.6.5 (2018-09-19)
------------------
* [bugfix] pick up missing feedback messages in inverters
* [bugfix] eliminate costly/spammy blackboard variable check feedback message

0.6.4 (2018-09-19)
------------------
* [bugfix] replace awkward newlines with spaces in ascii trees

0.6.3 (2018-09-04)
------------------
* [bugfix] don't send the parellel's status to running children, invalidate them instead

0.6.2 (2018-08-31)
------------------
* [bugfix] oneshot now reacts to priority interrupts correctly

0.6.1 (2018-08-20)
------------------
* [bugfix] oneshot no longer permanently modifies the original class

0.6.0 (2018-05-15)
------------------
* [infra] python 2/3 compatibility

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

.. _PR117: https://github.com/stonier/py_trees/pull/117

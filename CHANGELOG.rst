Release Notes
=============

Forthcoming
-----------
* ...

2.0.5 (2019-12-25)
------------------
* [display] enum agnostic display for local and remote activity view displays

2.0.4 (2019-11-25)
------------------
* [display] optional show_title in unicode_blackboard_activity_stream

2.0.3 (2019-11-24)
------------------
* [trees] revert to using user signals if available to avoid shenanigans with SIGINT, `#264 <https://github.com/splintered-reality/py_trees/pull/264>`_
* [trees] play nicely, reset signal handlers after setup, `#262 <https://github.com/splintered-reality/py_trees/pull/262>`_
* [visitors] bugfix the snapshot visitor to look for exclusive write keys as well

2.0.1 (2019-11-19)
------------------
* [blackboard] static methods have a namespace too (root), use absolute names, `#261 <https://github.com/splintered-reality/py_trees/pull/261>`_
* [blackboard] do not register keys on the client when xclusive write aborts the process, `#261 <https://github.com/splintered-reality/py_trees/pull/261>`_

2.0.x (2019-11-15) - Blackboards v2!
------------------------------------

The `2.0.x` release wraps up the experimental blackboard improvements being rolled out
in `1.3.x` and `1.4.x`. At this point, the changes to the blackboard framework are so
extensive it makes sense to release it with a major version bump and to consider the
`1.2.x` release as the official goto release for the `1.x.y` series.

**New Features**

* [blackboard] exclusive write access, `#260 <https://github.com/splintered-reality/py_trees/pull/260>`_
* [blackboard] key remappings, `#259 <https://github.com/splintered-reality/py_trees/pull/259>`_
* [blackboard] formalise namespaces with separators, `#256 <https://github.com/splintered-reality/py_trees/pull/256>`_
* [blackboard] distinguish primitives vs nested for refined read activity detection, `#255 <https://github.com/splintered-reality/py_trees/pull/255>`_

See the 1.3.x and 1.4.x changelog notes for additional details.

1.4.x (2019-11-07)
------------------

**Breaking API**

* [blackboard] fixed read/write ambiguity, now use ``py_trees.common.Access``, `#250 <https://github.com/splintered-reality/py_trees/pull/250>`_

.. code-block:: python

    # Previously
    self.blackboard.register_key(key="foo", write=True)
    # Now
    self.blackboard.register_key(key="foo", access=py_trees.common.Access.WRITE)

* [blackboard] drop ``SubBlackboard``, it has problems, `#249 <https://github.com/splintered-reality/py_trees/pull/249>`_

**New Features**

* [blackboard] namespaced blackboard clients, `#250 <https://github.com/splintered-reality/py_trees/pull/250>`_

.. code-block:: python

    # Previously, a single blackboard client exists per behaviour
    # Now, no blackboard client on construction, instead attach on demand:
    self.blackboard = self.attach_blackboard_client(name="Foo")
    self.parameters = self.attach_blackboard_client(
        name="FooParams",
        namespace="parameters_foo_"
    )
    self.state = self.attach_blackboard_client(
        name="FooState",
        namespace="state_foo_"
    )
    # create a local key 'speed' that maps to 'state_foo_speed' on the blackboard
    self.state.register_key(key="speed", access=py_trees.common.Access.WRITE)
    self.state.speed = 30.0

* [blackboard] required keys and batch verification method, `#254 <https://github.com/splintered-reality/py_trees/pull/254>`_

.. code-block:: python

    self.blackboard = self.attach_blackboard_client(name="Foo")
    self.blackboard.register_key(name="foo", access=py_trees.common.Access.READ, required=True)
    # ...
    self.verify_required_keys_exist()  # KeyError if any required keys do not yet exist on the blackboard

* [visitors] ``SnapshotVisitor`` tracking blackboards on the visited path, `#250 <https://github.com/splintered-reality/py_trees/pull/250>`_

.. code-block:: python

    # Previously tangled in DisplaySnapshotVisitor:
    display_snapshot_visitor.visited.keys()  # blackboard client uuid's (also behaviour uuid's), typing.Set[uuid.UUID]
    display_snapshot_visitor.visited_keys  # blackboard keys, typing.Set[str]
    # Now in SnapshotVisitor:
    snapshot_visitor.visited_blackboard_client_ids  # typing.Set[uuid.UUID]
    snapshot_visitor.visited_blackboard_keys  # typing.Set[str]


1.3.3 (2019-10-15)
------------------
* [blackboard] client ``Blackboard.unregister_key()`` method

1.3.2 (2019-10-15)
------------------
* [blackboard] global ``Blackboard.clear()`` method

1.3.1 (2019-10-15)
------------------
* [blackboard] don't do any copying, just pass handles around, `#239 <https://github.com/splintered-reality/py_trees/pull/239>`_
* [blackboard] client ``exists()`` method, `#238 <https://github.com/splintered-reality/py_trees/pull/238>`_
* [blackboard] global ``Blackboard.set()`` method
* [blackboard] client ``Blackboard.unset()`` method, `#239 <https://github.com/splintered-reality/py_trees/pull/239>`_

1.3.x (2019-10-03)
------------------

**Breaking API**

* [decorators] updated ``EternalGuard`` to accommodate new blackboard variable tracking mechanisms
* [behaviours] blackboard behaviours decoupled - ``CheckBlackboardVariableExists``, ``WaitForBlackboardVariable``
* [behaviours] blackboard behaviours decoupled - ``CheckBlackboardVariableValue``, ``WaitForBlackboardVariableValue``
* [behaviours] blackboard behaviours dropped use of the largely redundant ``ClearingPolicy``
* [visitors] collapsed ``SnapshotVisitor`` and ``WindsOfChangeVisitor`` functionality, `#228 <https://github.com/splintered-reality/py_trees/pull/228>`_

**New Features**

* [blackboard] read/write access configuration for clients on blackboard keys
* [blackboard] log the activity on the blackboard
* [display] dot graphs now have an option to display blackboard variables
* [display] unicode to console the entire blackboard key-value store
* [display] unicode to console the blackboard activity stream
* [visitors] new ``DisplaySnapshotVisitor`` to simplify collection/printing the tree to console, `#228 <https://github.com/splintered-reality/py_trees/pull/228>`_

**Bugfixes**

* [infra] only require test html reports on circle ci builds (saves a dependency requirement), `#229 <https://github.com/splintered-reality/py_trees/pull/229>`_

1.2.2 (2019-08-06)
------------------
* [trees] standalone ``setup()`` method with timer for use on unmanaged trees, `#198 <https://github.com/splintered-reality/py_trees/pull/198>`_
* [examples] fix api in ``skeleton_tree.py``,  `#199 <https://github.com/splintered-reality/py_trees/pull/199>`_

1.2.1 (2019-05-21)
------------------
* [decorators] ``StatusToBlackboard`` reflects the status of it's child to the blackboard, `#195 <https://github.com/splintered-reality/py_trees/pull/195>`_
* [decorators] ``EternalGuard`` decorator that continuously guards a subtree (c.f. Unreal conditions), `#195 <https://github.com/splintered-reality/py_trees/pull/195>`_
* [idioms] ``eternal_guard`` counterpart to the decorator whose conditions are behaviours, `#195 <https://github.com/splintered-reality/py_trees/pull/195>`_

1.2.x (2019-04-28)
------------------

**Breaking API**

* [trees] removes the curious looking and unused ``destroy()`` method, `#193 <https://github.com/splintered-reality/py_trees/pull/193>`_
* [display] ``ascii_tree`` -> ``ascii_tree``/``unicode_tree()``, no longer subverts the choice depending on your stdout, `#192 <https://github.com/splintered-reality/py_trees/pull/192>`_
* [display] ``dot_graph`` -> ``dot_tree`` for consistency with the text tree methods, `#192 <https://github.com/splintered-reality/py_trees/pull/192>`_

**New Features**

* [behaviour] ``shutdown()`` method to compliment ``setup()``, `#193 <https://github.com/splintered-reality/py_trees/pull/193>`_
* [decorators] ``StatusToBlackboard`` reflects the status of it's child to the blackboard, `#195 <https://github.com/splintered-reality/py_trees/pull/195>`_
* [decorators] ``EternalGuard`` decorator that continuously guards a subtree (c.f. Unreal conditions), `#195 <https://github.com/splintered-reality/py_trees/pull/195>`_
* [display] ``xhtml_tree`` provides an xhtml compatible equivalent to the ``ascii_tree`` representation, `#192 <https://github.com/splintered-reality/py_trees/pull/192>`_
* [idioms] ``eternal_guard`` counterpart to the decorator whose conditions are behaviours, `#195 <https://github.com/splintered-reality/py_trees/pull/195>`_
* [trees] walks the tree calling ``shutdown()`` on each node in it's own ``shutdown()`` method, `#193 <https://github.com/splintered-reality/py_trees/pull/193>`_
* [visitors] get a ``finalise()`` method called immediately prior to post tick handlers, `#191 <https://github.com/splintered-reality/py_trees/pull/191>`_

1.1.0 (2019-03-19)
------------------

**Breaking API**

* [display] print_ascii_tree -> ascii_tree, `#178 <https://github.com/splintered-reality/py_trees/pull/178>`_.
* [display] generate_pydot_graph -> dot_graph, `#178 <https://github.com/splintered-reality/py_trees/pull/178>`_.
* [trees] tick_tock(sleep_ms, ..) -> tick_tock(period_ms, ...),  `#182 <https://github.com/splintered-reality/py_trees/pull/182>`_.

**New Features**

* [trees] add missing ``add_visitor()`` method
* [trees] flexible ``setup()`` for children via kwargs
* [trees] convenience method for ascii tree debugging
* [display] highlight the tip in ascii tree snapshots

**Bugfixes**

* [trees] threaded timers for setup (avoids multiprocessing problems)
* [behaviour|composites] bugfix tip behaviour, add tests
* [display] correct first indent when non-zero in ascii_tree
* [display] apply same formatting to root as children in ascii_tree

1.0.7 (2019-xx-yy)
------------------
* [display] optional arguments for generate_pydot_graph

1.0.6 (2019-03-06)
------------------
* [decorators] fix missing root feedback message in ascii graphs

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

1.0.0 (2019-01-18)
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

.. _modules-section-label:

Module API
==========

py_trees
--------

.. automodule:: py_trees
   :synopsis: are your behaviour trees misbehaving?

py_trees.behaviour
------------------

.. automodule:: py_trees.behaviour
    :members:
    :show-inheritance:
    :synopsis: core template from which all behaviours are derived

py_trees.behaviours
-------------------

.. automodule:: py_trees.behaviours
    :members:
    :show-inheritance:
    :synopsis: library of useful behaviours

py_trees.blackboard
-------------------

.. automodule:: py_trees.blackboard
    :members:
    :special-members:
    :show-inheritance:
    :synopsis: shared data store and related behaviours

py_trees.common
---------------

.. automodule:: py_trees.common
    :synopsis: common definitions, methods and enumerations

.. autoclass:: py_trees.common.Access
   :members: READ, WRITE, EXCLUSIVE_WRITE
   :show-inheritance:

.. autoclass:: py_trees.common.BlackBoxLevel
    :members: BIG_PICTURE, COMPONENT, DETAIL, NOT_A_BLACKBOX
    :show-inheritance:

.. autoclass:: py_trees.common.ClearingPolicy
    :members: ON_INITIALISE, ON_SUCCESS, NEVER
    :show-inheritance:

.. autoclass:: py_trees.common.Duration
    :members: INFINITE, UNTIL_THE_BATTLE_OF_ALFREDO
    :show-inheritance:

.. autoclass:: py_trees.common.Name
    :members: AUTO_GENERATED
    :show-inheritance:

.. autoclass:: py_trees.common.ParallelPolicy
    :members: SuccessOnAll, SuccessOnOne, SuccessOnSelected

.. autoclass:: py_trees.common.Status
    :members: SUCCESS, FAILURE, RUNNING, INVALID
    :show-inheritance:

.. autoclass:: py_trees.common.VisibilityLevel
    :members: ALL, DETAIL, COMPONENT, BIG_PICTURE
    :show-inheritance:

.. automethod:: py_trees.common.string_to_visibility_level

.. _py-trees-composites-module:

py_trees.composites
-------------------

.. automodule:: py_trees.composites
    :members:
    :special-members:
    :show-inheritance:
    :synopsis: behaviours that have children

py_trees.console
----------------

.. automodule:: py_trees.console
    :members:
    :synopsis: colour definitions and syntax highlighting for the console

py_trees.decorators
-------------------

.. automodule:: py_trees.decorators
    :members:
    :show-inheritance:
    :synopsis: hats for behaviours

py_trees.display
----------------

.. automodule:: py_trees.display
    :members:
    :show-inheritance:
    :synopsis: visualising trees with dot graphs, strings or on stdout

.. _py-trees-idioms-module:

py_trees.idioms
---------------

.. automodule:: py_trees.idioms
    :members:
    :special-members:
    :show-inheritance:
    :synopsis: creators of common behaviour tree patterns

py_trees.meta
-------------

.. automodule:: py_trees.meta
    :members:
    :special-members:
    :show-inheritance:
    :synopsis: factories for behaviours

py_trees.timers
---------------

.. automodule:: py_trees.timers
    :members:
    :special-members:
    :show-inheritance:
    :synopsis: timer related behaviours

py_trees.trees
--------------

.. automodule:: py_trees.trees
    :members:
    :show-inheritance:
    :synopsis: tree managers - they make your life easier!

py_trees.utilities
------------------

.. automodule:: py_trees.utilities
    :members:
    :show-inheritance:
    :synopsis: assorted utility functions

py_trees.visitors
-----------------

.. automodule:: py_trees.visitors
    :members:
    :show-inheritance:
    :synopsis: entities that visit behaviours as a tree is traversed



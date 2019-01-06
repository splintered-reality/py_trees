#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/stonier/py_trees/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
.. attention::
    This module is the least likely to remain stable in this package. It has
    only received cursory attention so far and a more thoughtful design for handling
    behaviour 'hats' might be needful at some point in the future.

Meta behaviours are created by utilising various programming techniques
pulled from a magic bag of tricks. Some of these minimise the effort to generate
a new behaviour while others provide mechanisms that greatly expand your
library of usable behaviours without having to increase the number of explicit
behaviours contained therein. The latter is achieved by providing a means for
behaviours to wear different 'hats' via python decorators.

.. image:: images/many-hats.png
   :width: 40px
   :align: center

Each function or decorator listed below includes its own example code
demonstrating its use.

**Factories**

* :func:`py_trees.meta.create_behaviour_from_function`
* :func:`py_trees.meta.create_imposter`

**Decorators (Hats)**

* :func:`py_trees.meta.condition`
* :func:`py_trees.meta.inverter`
* :func:`py_trees.meta.failure_is_running`
* :func:`py_trees.meta.failure_is_success`
* :func:`py_trees.meta.oneshot`
* :func:`py_trees.meta.running_is_failure`
* :func:`py_trees.meta.running_is_success`
* :func:`py_trees.meta.success_is_failure`
* :func:`py_trees.meta.success_is_running`
* :func:`py_trees.meta.timeout`
"""

##############################################################################
# Imports
##############################################################################

import functools
import time

from . import behaviour
from . import common
from . import composites

##############################################################################
# Utility Methods
##############################################################################


def create_behaviour_from_function(func):
    """
    Create a behaviour from the specified function, dropping it in for
    the Behaviour :meth:`~py_trees.behaviour.Behaviour.update` method.
    Ths function must include the `self`
    argument and return a :class:`~py_trees.behaviours.common.Status` value.
    It also automatically provides a drop-in for the :meth:`~py_trees.behaviour.Behaviour.terminate`
    method that clears the feedback message. Other methods are left untouched.

    Args:
        func (:obj:`function`):  a drop-in for the :meth:`~py_trees.behaviour.Behaviour.update` method
    """
    class_name = func.__name__.capitalize()

    def init(self, name=class_name):
        behaviour.Behaviour.__init__(self, name=name)

    def terminate(self, new_status):
        if new_status == common.Status.INVALID:
            self.feedback_message = ""

    return type(class_name, (behaviour.Behaviour,), dict(__init__=init, update=func, terminate=terminate))

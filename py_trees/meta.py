#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/splintered-reality/py_trees/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""Meta methods to create behaviours without creating behaviours themselves."""

##############################################################################
# Imports
##############################################################################

import typing

from . import behaviour, common

##############################################################################
# Utility Methods
##############################################################################


# BehaviourUpdateMethod = typing.Callable[[BehaviourSubClass], common.Status]
# BehaviourUpdateMethod = typing.TypeVar(
#    'BehaviourUpdateMethod',
#    bound=typing.Callable[[behaviour.BehaviourSubClass], common.Status]
# )

BehaviourUpdateMethod = typing.TypeVar("BehaviourUpdateMethod", bound=typing.Callable)


def create_behaviour_from_function(
    func: BehaviourUpdateMethod, module: typing.Optional[str] = None
) -> "typing.Type[behaviour.Behaviour]":
    """
    Create a behaviour from the specified function.

    This takes the specified function and drops it in to serve as the
    the Behaviour :meth:`~py_trees.behaviour.Behaviour.update` method.

    The user provided fucntion must include the `self`
    argument and return a :class:`~py_trees.behaviours.common.Status` value.

    It also automatically registers a method for the :meth:`~py_trees.behaviour.Behaviour.terminate`
    method that clears the feedback message. Other methods are left untouched.

    Args:
        func: a drop-in for the :meth:`~py_trees.behaviour.Behaviour.update` method
        module: suppliment it with a __module__ name if required (otherwise it will default to 'abc.')
    """
    class_name = func.__name__.capitalize()

    def init(self: behaviour.Behaviour, name: str = class_name) -> None:
        behaviour.Behaviour.__init__(self, name=name)

    def terminate(self: behaviour.Behaviour, new_status: common.Status) -> None:
        if new_status == common.Status.INVALID:
            self.feedback_message = ""

    class_type = type(
        class_name,
        (behaviour.Behaviour,),
        dict(__init__=init, update=func, terminate=terminate),
    )

    # When module is None, it will default to 'abc.' since behaviour.Behaviour is an ABC.
    # If that does matter (e.g. you're creating a class for an actual module, not a script), then
    # use the module argument. NB: this is better than relying on magic inspect methods that aren't
    # consistently available across different python implementations.
    if module is not None:
        class_type.__module__ = module
    return class_type

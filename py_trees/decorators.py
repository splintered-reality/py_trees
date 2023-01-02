#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/splintered-reality/py_trees/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
Decorate your children. They make great furniture pieces.

Decorators are behaviours that manage a single child and provide common
modifications to their underlying child behaviour (e.g. inverting the result).
That is, they provide a means for behaviours to wear different 'hats' and
this combinatorially expands the capabilities of your behaviour library.

.. image:: images/many-hats.png
   :width: 40px
   :align: center

An example:

.. graphviz:: dot/decorators.dot
   :align: center

.. literalinclude:: examples/decorators.py
   :language: python
   :linenos:


**Decorators (Hats)**

Decorators with specific functionality:

* :class:`py_trees.decorators.Condition`
* :class:`py_trees.decorators.EternalGuard`
* :class:`py_trees.decorators.Inverter`
* :class:`py_trees.decorators.OneShot`
* :class:`py_trees.decorators.Repeat`
* :class:`py_trees.decorators.Retry`
* :class:`py_trees.decorators.StatusToBlackboard`
* :class:`py_trees.decorators.Timeout`

And the X is Y family:

* :class:`py_trees.decorators.FailureIsRunning`
* :class:`py_trees.decorators.FailureIsSuccess`
* :class:`py_trees.decorators.RunningIsFailure`
* :class:`py_trees.decorators.RunningIsSuccess`
* :class:`py_trees.decorators.SuccessIsFailure`
* :class:`py_trees.decorators.SuccessIsRunning`

**Decorators for Blocking Behaviours**

It is worth making a note of the effect of decorators on
blocking behaviours, i.e. those that return :data:`~py_trees.common.Status.RUNNING`
before eventually returning  :data:`~py_trees.common.Status.SUCCESS`
or  :data:`~py_trees.common.Status.FAILURE`.

A decorator, such as :func:`py_trees.decorators.RunningIsSuccess` on
a blocking behaviour will immediately terminate the underlying child and
re-intialise on it's next tick. This is often surprising (to the user) but
is necessary to ensure the underlying child isn't left in a dangling state (i.e.
:data:`~py_trees.common.Status.RUNNING`) as subsequent ticks move on to other
parts of the tree.

A better approach in this case is to build a non-blocking variant or a
combination of non-blocking behaviors that handle construction,
monitoring and destruction of the activity represented by the
original blocking behaviour.
"""

##############################################################################
# Imports
##############################################################################

import functools
import inspect
import time
import typing

from typing import Callable, List, Set, Union  # noqa

from . import behaviour
from . import blackboard
from . import common

##############################################################################
# Classes
##############################################################################


class Decorator(behaviour.Behaviour):
    """
    Parent class for decorating a child/subtree with some additional logic.

    Args:
        child: the child to be decorated
        name: the decorator name

    Raises:
        TypeError: if the child is not an instance of :class:`~py_trees.behaviour.Behaviour`
    """

    def __init__(
            self,
            child: behaviour.Behaviour,
            name=common.Name.AUTO_GENERATED
    ):
        # Checks
        if not isinstance(child, behaviour.Behaviour):
            raise TypeError("A decorator's child must be an instance of py_trees.behaviours.Behaviour")
        # Initialise
        super().__init__(name=name)
        self.children.append(child)
        # Give a convenient alias
        self.decorated = self.children[0]
        self.decorated.parent = self

    def tick(self):
        """
        Manage the decorated child through the tick.

        Yields:
            :class:`~py_trees.behaviour.Behaviour`: a reference to itself or one of its children
        """
        self.logger.debug("%s.tick()" % self.__class__.__name__)
        # initialise just like other behaviours/composites
        if self.status != common.Status.RUNNING:
            self.initialise()
        # interrupt proceedings and process the child node
        # (including any children it may have as well)
        for node in self.decorated.tick():
            yield node
        # resume normal proceedings for a Behaviour's tick
        new_status = self.update()
        if new_status not in list(common.Status):
            self.logger.error(
                "A behaviour returned an invalid status, setting to INVALID [%s][%s]" % (new_status, self.name)
            )
            new_status = common.Status.INVALID
        if new_status != common.Status.RUNNING:
            self.stop(new_status)
        self.status = new_status
        yield self

    def stop(self, new_status):
        """
        Check if the child is running (dangling) and stop it if that is the case.

        Args:
            new_status (:class:`~py_trees.common.Status`): the behaviour is transitioning to this new status
        """
        self.logger.debug("%s.stop(%s)" % (self.__class__.__name__, new_status))
        self.terminate(new_status)
        # priority interrupt handling
        if new_status == common.Status.INVALID:
            self.decorated.stop(new_status)
        # if the decorator returns SUCCESS/FAILURE and should stop the child
        if self.decorated.status == common.Status.RUNNING:
            self.decorated.stop(common.Status.INVALID)
        self.status = new_status

    def tip(self):
        """
        Retrieve the *tip* of this behaviour's subtree (if it has one).

        This corresponds to the the deepest node that was running before the
        subtree traversal reversed direction and headed back to this node.

        Returns:
            :class:`~py_trees.behaviour.Behaviour` or :obj:`None`: child behaviour,
                itself or :obj:`None` if its status is :data:`~py_trees.common.Status.INVALID`
        """
        if self.decorated.status != common.Status.INVALID:
            return self.decorated.tip()
        else:
            return super().tip()

##############################################################################
# Decorators
##############################################################################


class Repeat(Decorator):
    """
    Repeat.

    :data:`~py_trees.common.Status.SUCCESS` is
    :data:`~py_trees.common.Status.RUNNING` up to a specified number at
    which point this decorator returns :data:`~py_trees.common.Status.SUCCESS`.

    :data:`~py_trees.common.Status.FAILURE` is always
    :data:`~py_trees.common.Status.FAILURE`.

    Args:
        child: the child behaviour or subtree
        num_success: repeat this many times (-1 to repeat indefinitely)
        name: the decorator name
    """

    def __init__(
        self,
        name: str,
        num_success: int,
        child: behaviour.Behaviour
    ):
        super().__init__(child=child, name=name)
        self.success = 0
        self.num_success = num_success

    def initialise(self):
        """
        Reset the currently registered number of successes.
        """
        self.success = 0

    def update(self):
        if self.decorated.status == common.Status.FAILURE:
            self.feedback_message = f"failed, aborting [status: {self.success} success from {self.num_success}]"
            return common.Status.FAILURE
        elif self.decorated.status == common.Status.SUCCESS:
            self.success += 1
            self.feedback_message = f"success [status: {self.success} success from {self.num_success}]"
            if self.success == self.num_success:
                return common.Status.SUCCESS
            else:
                return common.Status.RUNNING
        else:  # RUNNING
            self.feedback_message = f"running [status: {self.success} success from {self.num_success}]"
            return common.Status.RUNNING


class Retry(Decorator):
    """
    Keep trying, pastafarianism is within reach.

    :data:`~py_trees.common.Status.FAILURE` is
    :data:`~py_trees.common.Status.RUNNING` up to a specified number of
    attempts.

    Args:
        child: the child behaviour or subtree
        num_failures: maximum number of permitted failures
        name: the decorator name
    """

    def __init__(
        self,
        name: str,
        num_failures: int,
        child: behaviour.Behaviour
    ):
        super().__init__(child=child, name=name)
        self.failures = 0
        self.num_failures = num_failures

    def initialise(self):
        """
        Reset the currently registered number of attempts.
        """
        self.failures = 0

    def update(self):
        if self.decorated.status == common.Status.FAILURE:
            self.failures += 1
            if self.failures < self.num_failures:
                self.feedback_message = f"attempt failed [status: {self.failures} failure from {self.num_failures}]"
                return common.Status.RUNNING
            else:
                self.feedback_message = f"final failure [status: {self.failures} failure from {self.num_failures}]"
                return common.Status.FAILURE
        elif self.decorated.status == common.Status.RUNNING:
            self.feedback_message = f"running [status: {self.failures} failure from {self.num_failures}]"
            return common.Status.RUNNING
        else:  # SUCCESS
            self.feedback_message = f"succeeded [status: {self.failures} failure from {self.num_failures}]"
            return common.Status.SUCCESS


class StatusToBlackboard(Decorator):
    """
    Reflect the status of the decorator's child to the blackboard.

    Args:
        child: the child behaviour or subtree
        variable_name: name of the blackboard variable, may be nested, e.g. foo.status
        name: the decorator name
    """

    def __init__(
            self,
            *,
            child: behaviour.Behaviour,
            variable_name: str,
            name: Union[str, common.Name] = common.Name.AUTO_GENERATED,
    ):
        super().__init__(child=child, name=name)
        self.variable_name = variable_name
        name_components = variable_name.split('.')
        self.key = name_components[0]
        self.key_attributes = '.'.join(name_components[1:])  # empty string if no other parts
        self.blackboard = self.attach_blackboard_client(self.name)
        self.blackboard.register_key(key=self.key, access=common.Access.WRITE)

    def update(self):
        """
        Reflect the decorated child's status to the blackboard.

        Returns: the decorated child's status
        """
        self.blackboard.set(
            name=self.variable_name,
            value=self.decorated.status,
            overwrite=True
        )
        return self.decorated.status


ConditionType = Union[
    Callable[[], bool],
    Callable[[], common.Status],
    Callable[[blackboard.Blackboard], bool],
    Callable[[blackboard.Blackboard], common.Status]
]


class EternalGuard(Decorator):
    """
    Continuously guard (with a condition) the execution of a child/subtree.

    A decorator that continually guards the execution of a subtree.
    If at any time the guard's condition check fails, then the child
    behaviour/subtree is invalidated.

    .. note:: This decorator's behaviour is stronger than the
       :term:`guard` typical of a conditional check at the beginning of a
       sequence of tasks as it continues to check on every tick whilst the
       task (or sequence of tasks) runs.

    Args:
        child: the child behaviour or subtree
        condition: a functional check that determines execution or not of the subtree
        blackboard_keys: provide read access for the conditional function to these keys
        name: the decorator name

    Examples:
        Simple conditional function returning True/False:

        .. code-block:: python

            def check():
                 return True

            foo = py_trees.behaviours.Foo()
            eternal_guard = py_trees.decorators.EternalGuard(
                name="Eternal Guard",
                condition=check,
                child=foo
            )

        Simple conditional function returning SUCCESS/FAILURE:

        .. code-block:: python

            def check():
                 return py_trees.common.Status.SUCCESS

            foo = py_trees.behaviours.Foo()
            eternal_guard = py_trees.decorators.EternalGuard(
                name="Eternal Guard",
                condition=check,
                child=foo
            )

        Conditional function that makes checks against data on the blackboard (the
        blackboard client with pre-configured access is provided by the EternalGuard
        instance):

        .. code-block:: python

            def check(blackboard):
                 return blackboard.velocity > 3.0

            foo = py_trees.behaviours.Foo()
            eternal_guard = py_trees.decorators.EternalGuard(
                name="Eternal Guard",
                condition=check,
                blackboard_keys={"velocity"},
                child=foo
            )

    .. seealso:: :meth:`py_trees.idioms.eternal_guard`
    """

    def __init__(
            self,
            *,
            child: behaviour.Behaviour,
            # Condition is one of 4 callable types illustrated in the docstring, partials complicate
            # it as well. When typing_extensions are available (very recent) more generally, can use
            # Protocols to handle it. Probably also a sign that it's not a very clean api though...
            condition: typing.Any,
            blackboard_keys: Union[List[str], Set[str]] = None,
            name: Union[str, common.Name] = common.Name.AUTO_GENERATED,
    ):
        if blackboard_keys is None:
            blackboard_keys = []
        super().__init__(name=name, child=child)
        self.blackboard = self.attach_blackboard_client(self.name)
        for key in blackboard_keys:
            self.blackboard.register_key(key=key, access=common.Access.READ)
        condition_signature = inspect.signature(condition)
        if "blackboard" in [p.name for p in condition_signature.parameters.values()]:
            self.condition = functools.partial(condition, self.blackboard)
        else:
            self.condition = condition

    def tick(self):
        """
        Conditionally manage the child.

        Yields:
            :class:`~py_trees.behaviour.Behaviour`: a reference to itself or one of its children
        """
        self.logger.debug("%s.tick()" % self.__class__.__name__)

        # condition check
        result = self.condition()
        if type(result) == common.Status:
            result = False if result == common.Status.FAILURE else True
        elif type(result) != bool:
            error_message = "conditional check must return 'bool' or 'common.Status' [{}]".format(type(result))
            self.logger.error("The {}".format(error_message))
            raise RuntimeError(error_message)

        if not result:
            # abort, abort, the FSM is losing his noodles!!!
            self.stop(common.Status.FAILURE)
            yield self
        else:
            # normal behaviour
            for node in super().tick():
                yield node

    def update(self):
        """
        Reflect the decorated child's status.

        The update method is only ever triggered in the child's post-tick, which implies
        that the condition has already been checked and passed (refer to the :meth:`tick` method).
        """
        return self.decorated.status


class Timeout(Decorator):
    """
    Executes a child/subtree with a timeout.

    A decorator that applies a timeout pattern to an existing behaviour.
    If the timeout is reached, the encapsulated behaviour's
    :meth:`~py_trees.behaviour.Behaviour.stop` method is called with
    status :data:`~py_trees.common.Status.FAILURE` otherwise it will
    simply directly tick and return with the same status
    as that of it's encapsulated behaviour.
    """

    def __init__(self,
                 child: behaviour.Behaviour,
                 name: Union[str, common.Name] = common.Name.AUTO_GENERATED,
                 duration: float = 5.0):
        """
        Init with the decorated child and a timeout duration.

        Args:
            child: the child behaviour or subtree
            name: the decorator name
            duration: timeout length in seconds
        """
        super(Timeout, self).__init__(name=name, child=child)
        self.duration = duration
        self.finish_time = None

    def initialise(self):
        """
        Reset the feedback message and finish time on behaviour entry.
        """
        self.finish_time = time.monotonic() + self.duration
        self.feedback_message = ""

    def update(self):
        """
        Handle a timeout with failure, otherwise block / reflect the child's result accordingly.

        Terminate the child and return :data:`~py_trees.common.Status.FAILURE`
        if the timeout is exceeded.
        """
        current_time = time.monotonic()
        if self.decorated.status == common.Status.RUNNING and current_time > self.finish_time:
            self.feedback_message = "timed out"
            self.logger.debug("{}.update() {}".format(self.__class__.__name__, self.feedback_message))
            # invalidate the decorated (i.e. cancel it), could also put this logic in a terminate() method
            self.decorated.stop(common.Status.INVALID)
            return common.Status.FAILURE
        if self.decorated.status == common.Status.RUNNING:
            self.feedback_message = "time still ticking ... [remaining: {}s]".format(
                self.finish_time - current_time
            )
        else:
            self.feedback_message = "child finished before timeout triggered"
        return self.decorated.status


class OneShot(Decorator):
    """
    A decorator that implements the oneshot pattern.

    This decorator ensures that the underlying child is ticked through
    to completion just once and while doing so, will return
    with the same status as it's child. Thereafter it will return
    with the final status of the underlying child.

    Completion status is determined by the policy given on construction.

    * With policy :data:`~py_trees.common.OneShotPolicy.ON_SUCCESSFUL_COMPLETION`, the oneshot will activate
      only when the underlying child returns :data:`~py_trees.common.Status.SUCCESS` (i.e. it permits retries).
    * With policy :data:`~py_trees.common.OneShotPolicy.ON_COMPLETION`, the oneshot will activate when the child
      returns :data:`~py_trees.common.Status.SUCCESS` || :data:`~py_trees.common.Status.FAILURE`.

    .. seealso:: :meth:`py_trees.idioms.oneshot`
    """

    def __init__(self,
                 child: behaviour.Behaviour,
                 name: typing.Union[str, common.Name] = common.Name.AUTO_GENERATED,
                 policy: common.OneShotPolicy = common.OneShotPolicy.ON_SUCCESSFUL_COMPLETION):
        """
        Init with the decorated child.

        Args:
            child: behaviour to shoot
            name: the decorator name
            policy: policy determining when the oneshot should activate
        """
        super(OneShot, self).__init__(name=name, child=child)
        self.final_status = None
        self.policy = policy

    def update(self):
        """
        Bounce if the child has already successfully completed.
        """
        if self.final_status:
            self.logger.debug("{}.update()[bouncing]".format(self.__class__.__name__))
            return self.final_status
        return self.decorated.status

    def tick(self):
        """
        Tick the child or bounce back with the original status if already completed.
        """
        if self.final_status:
            # ignore the child
            for node in behaviour.Behaviour.tick(self):
                yield node
        else:
            # tick the child
            for node in Decorator.tick(self):
                yield node

    def terminate(self, new_status):
        """
        Prevent further entry if finishing with :data:`~py_trees.common.Status.SUCCESS`.

        This uses a flag to register that the behaviour has gone through to completion.
        In future ticks, it will block entry to the child and just return the original
        status result.
        """
        if not self.final_status and new_status in self.policy.value:
            self.logger.debug("{}.terminate({})[oneshot completed]".format(self.__class__.__name__, new_status))
            self.feedback_message = "oneshot completed"
            self.final_status = new_status
        else:
            self.logger.debug("{}.terminate({})".format(self.__class__.__name__, new_status))


class Inverter(Decorator):
    """
    A decorator that inverts the result of a class's update function.
    """

    def __init__(self, child, name=common.Name.AUTO_GENERATED):
        """
        Init with the decorated child.

        Args:
            child (:class:`~py_trees.behaviour.Behaviour`): behaviour to time
            name (:obj:`str`): the decorator name
        """
        super(Inverter, self).__init__(name=name, child=child)

    def update(self):
        """
        Flip :data:`~py_trees.common.Status.SUCCESS` and :data:`~py_trees.common.Status.FAILURE`.

        Returns:
            :class:`~py_trees.common.Status`: the behaviour's new status :class:`~py_trees.common.Status`
        """
        if self.decorated.status == common.Status.SUCCESS:
            self.feedback_message = "success -> failure"
            return common.Status.FAILURE
        elif self.decorated.status == common.Status.FAILURE:
            self.feedback_message = "failure -> success"
            return common.Status.SUCCESS
        self.feedback_message = self.decorated.feedback_message
        return self.decorated.status


class RunningIsFailure(Decorator):
    """
    Got to be snappy! We want results...yesterday.
    """

    def update(self):
        """
        Reflect :data:`~py_trees.common.Status.RUNNING` as :data:`~py_trees.common.Status.FAILURE`.

        Returns:
            :class:`~py_trees.common.Status`: the behaviour's new status :class:`~py_trees.common.Status`
        """
        if self.decorated.status == common.Status.RUNNING:
            self.feedback_message = "running is failure" \
                + (" [%s]" % self.decorated.feedback_message if self.decorated.feedback_message else "")
            return common.Status.FAILURE
        else:
            self.feedback_message = self.decorated.feedback_message
            return self.decorated.status


class RunningIsSuccess(Decorator):
    """
    Don't hang around...
    """

    def update(self):
        """
        Reflect :data:`~py_trees.common.Status.RUNNING` as :data:`~py_trees.common.Status.SUCCESS`.

        Returns:
            :class:`~py_trees.common.Status`: the behaviour's new status :class:`~py_trees.common.Status`
        """
        if self.decorated.status == common.Status.RUNNING:
            self.feedback_message = "running is success" \
                + (" [%s]" % self.decorated.feedback_message if self.decorated.feedback_message else "")
            return common.Status.SUCCESS
        self.feedback_message = self.decorated.feedback_message
        return self.decorated.status


class FailureIsSuccess(Decorator):
    """
    Be positive, always succeed.
    """

    def update(self):
        """
        Reflect :data:`~py_trees.common.Status.FAILURE` as :data:`~py_trees.common.Status.SUCCESS`.

        Returns:
            :class:`~py_trees.common.Status`: the behaviour's new status :class:`~py_trees.common.Status`
        """
        if self.decorated.status == common.Status.FAILURE:
            self.feedback_message = "failure is success" \
                + (" [%s]" % self.decorated.feedback_message if self.decorated.feedback_message else "")
            return common.Status.SUCCESS
        self.feedback_message = self.decorated.feedback_message
        return self.decorated.status


class FailureIsRunning(Decorator):
    """
    Dont stop running.
    """

    def update(self):
        """
        Reflect :data:`~py_trees.common.Status.FAILURE` as :data:`~py_trees.common.Status.RUNNING`.

        Returns:
            :class:`~py_trees.common.Status`: the behaviour's new status :class:`~py_trees.common.Status`
        """
        if self.decorated.status == common.Status.FAILURE:
            self.feedback_message = "failure is running" \
                + (" [%s]" % self.decorated.feedback_message if self.decorated.feedback_message else "")
            return common.Status.RUNNING
        self.feedback_message = self.decorated.feedback_message
        return self.decorated.status


class SuccessIsFailure(Decorator):
    """
    Be depressed, always fail.
    """

    def update(self):
        """
        Reflect :data:`~py_trees.common.Status.SUCCESS` as :data:`~py_trees.common.Status.FAILURE`.

        Returns:
            :class:`~py_trees.common.Status`: the behaviour's new status :class:`~py_trees.common.Status`
        """
        if self.decorated.status == common.Status.SUCCESS:
            self.feedback_message = "success is failure" \
                + (" [%s]" % self.decorated.feedback_message if self.decorated.feedback_message else "")
            return common.Status.FAILURE
        self.feedback_message = self.decorated.feedback_message
        return self.decorated.status


class SuccessIsRunning(Decorator):
    """
    It never ends...
    """

    def update(self):
        """
        Reflect :data:`~py_trees.common.Status.SUCCESS` as :data:`~py_trees.common.Status.RUNNING`.

        Returns:
            :class:`~py_trees.common.Status`: the behaviour's new status :class:`~py_trees.common.Status`
        """
        if self.decorated.status == common.Status.SUCCESS:
            self.feedback_message = "success is running [%s]" % self.decorated.feedback_message
            return common.Status.RUNNING
        self.feedback_message = self.decorated.feedback_message
        return self.decorated.status


class Condition(Decorator):
    """
    A blocking conditional decorator.

    Encapsulates a behaviour and wait for it's status to flip to the
    desired state. This behaviour will tick with
    :data:`~py_trees.common.Status.RUNNING` while waiting and
    :data:`~py_trees.common.Status.SUCCESS` when the flip occurs.
    """

    def __init__(
        self,
        child: behaviour.Behaviour,
        name: typing.Union[str, common.Name] = common.Name.AUTO_GENERATED,
        status: common.Status = common.Status.SUCCESS
    ):
        """
        Initialise with child and optional name, status variables.

        Args:
            child: the child to be decorated
            name: the decorator name (can be None)
            status: the desired status to watch for
        """
        super(Condition, self).__init__(child, name)
        self.succeed_status = status

    def update(self):
        """
        Check if the condtion has triggered, block otherwise.

        :data:`~py_trees.common.Status.SUCCESS` if the decorated child has returned
        the specified status, otherwise :data:`~py_trees.common.Status.RUNNING`.
        This decorator will never return :data:`~py_trees.common.Status.FAILURE`

        Returns:
            :class:`~py_trees.common.Status`: the behaviour's new status :class:`~py_trees.common.Status`
        """
        self.logger.debug("%s.update()" % self.__class__.__name__)
        self.feedback_message = (
            f"'{self.decorated.name}' has status {self.decorated.status}, waiting for {self.succeed_status}"
        )
        if self.decorated.status == self.succeed_status:
            return common.Status.SUCCESS
        return common.Status.RUNNING

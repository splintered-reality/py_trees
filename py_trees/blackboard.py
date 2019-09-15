#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# License: BSD
#   https://raw.githubusercontent.com/splintered-reality/py_trees/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
Blackboards are not a necessary component of behaviour tree implementations,
but are nonetheless, a fairly common mechanism for for sharing data between
behaviours in the tree. See, for example, the `design notes`_
for blackboards in Unreal Engine.

.. image:: images/blackboard.jpg
   :width: 300px
   :align: center

Implementations vary widely depending on the needs of
the framework using them. The simplest implementations take the
form of a key-value store with global access, while more
rigorous implementations scope access and form a secondary
graph overlaying the tree graph connecting data ports between behaviours.

The implementation here strives to remain simple to use
(so 'rapid development' does not become just 'development'), yet
sufficiently featured so that the magic behind the scenes (i.e. the
data sharing on the blackboard) is exposed and helpful in debugging
tree applications.

To be more concrete, the following is a list of features that this
implementation either embraces or does not.

* [+] Centralised key-value store
* [+] Client based usage with declaration of read/write intentions at construction
* [+] Activity stream that tracks read/write operations by behaviours
* [-] Sharing between tree instances
* [-] Locking for reading/writing
* [-] Strict variable initialisation policies

.. include:: weblinks.rst
"""

##############################################################################
# Imports
##############################################################################

import enum
import re
import operator
import typing
import uuid

from . import behaviour
from . import behaviours
from . import common
from . import console

##############################################################################
# Classes
##############################################################################


class KeyMetaData(object):

    def __init__(self):
        self.read = set()
        self.write = set()


class ActivityType(enum.Enum):
    """An enumerator representing the operation on a blackboard variable"""

    READ = "READ"
    """Read from the blackboard"""
    INITIALISED = "INITIALISED"
    """Initialised value on the blackboard"""
    WRITE = "WRITE"
    """Wrote to the blackboard."""
    READ_FAILED = "READ_FAILED"
    """Tried to read a key that does not yet exist on the blackboard."""
    READ_DENIED = "READ_DENIED"
    """Client was denied access to read a key."""
    WRITE_DENIED = "WRITE_DENIED"
    """Client was denied access to write a key."""
    NO_OVERWRITE = "NO_OVERWRITE"
    """Variable already exists and a no-overwrite request was respected."""
    UNSET = "UNSET"
    """Key was removed from the blackboard"""


class ActivityItem(object):
    """
    Recorded data pertaining to activity on the blackboard.

    Args:
        key: name of the variable on the blackboard
        client_name: convenient name of the client performing the operation
        client_id: unique id of the client performing the operation
        activity_type: type of activity
        previous_value: of the given key (None if this field is not relevant)
        current_value: current value for the given key (None if this field is not relevant)
    """
    def __init__(
            self,
            key,
            client_name: str,
            client_id: uuid.UUID,
            activity_type: ActivityType,
            previous_value: typing.Any=None,
            current_value: typing.Any=None):
        # TODO validity checks for values passed/not passed on the
        # respective activity types. Note: consider using an enum
        # for 'no value here' since None is a perfectly acceptable
        # value for a key
        self.key = key
        self.client_name = client_name
        self.client_id = client_id
        self.activity_type = activity_type
        self.previous_value = previous_value
        self.current_value = current_value


class ActivityStream(object):
    """
    Storage container with convenience methods for manipulating the stored
    activity stream.

    Attributes:
        data (typing.List[ActivityItem]: list of activity items, earliest first
        maximum_size (int): pop items if this size is exceeded
    """

    def __init__(self, maximum_size: int=500):
        """
        Initialise the stream with a maximum storage limit.

        Args:
            maximum_size: pop items from the stream if this size is exceeded
        """
        self.data = []
        self.maximum_size = maximum_size

    def push(self, activity_item: ActivityItem):
        """
        Push the next activity item to the stream.

        Args:
            activity_item: new item to append to the stream
        """
        if len(self.data) > self.maximum_size:
            self.data.pop()
        self.data.append(activity_item)

    def clear(self):
        """
        Delete all activities from the stream.
        """
        self.data = []


class Blackboard(object):
    """
    Key-value store for sharing data between behaviours.

    Examples:
        You can instantiate a blackboard client anywhere in your program.
        Even disconnected instantiations will discover the single global key-value store.
        For example:

        .. code-block:: python

            def check_foo():
                blackboard = Blackboard(name="Reader", read={"foo"))
                assert(blackboard.foo, "bar")

            if __name__ == '__main__':
                blackboard = Blackboard(name="Writer", write={"foo"))
                blackboard.foo = "bar"
                check_foo()

        To respect an already initialised key on the blackboard:

        .. code-block:: python

            if __name__ == '__main__':
                blackboard = Blackboard(name="Writer", read={"foo"))
                result = blackboard.set("foo", "bar", overwrite=False)

        Log and display the activity stream:

        .. code-block:: python

            # all key-value pairs
            Blackboard.enable_activity_stream(maximum_size=100)
            blackboard = Blackboard(name="Writer", write={"foo"))
            blackboard.foo = "bar"
            # view the activity stream
            print(py_trees.display.unicode_blackboard_activity_stream())
            # output
            Activity Stream
                foo       : INITIALISED  | Set Foo   | → bar
            # clear the stream (useful to do between tree ticks)
            Blackboard.activity_stream.clear()

        For introspection, use the methods in the display module:

        .. code-block:: python

            # all key-value pairs
            print(py_trees.display.unicode_blackboard())
            # various filtered views
            print(py_trees.display.unicode_blackboard(key_filter={"foo"}))
            print(py_trees.display.unicode_blackboard(regex_filter="dud*"))
            print(py_trees.display.unicode_blackboard(client_filter={writer.id, set_foo.id}))
            # list the clients associated with each key
            print(py_trees.display.unicode_blackboard(display_only_key_metadata=True)
            # a dot graph representation of tree and blackboard together
            py_trees.display.render_dot_tree(root, root, with_blackboard_variables=True)

    .. warning::

       Be careful of key collisions. This implementation leaves this management up to the user.

    .. seealso::
       * :ref:`py-trees-demo-blackboard <py-trees-demo-blackboard-program>`

    Attributes:
        <class>.storage (typing.Dict[str, typing.Any]): key-value storage
        <class>.metadata (typing.Dict[str, KeyMetaData]): key-client (read/write) metadata
        <class>.activity_stream (ActivityStream): the activity stream (None if not enabled)
        <instance>.name (str): client's convenient, but not necessarily unique identifier
        <instance>.unique_identifier (uuid.UUID): client's unique identifier
        <instance>.read (typing.List[str]): keys this client has permission to read
        <instance>.write (typing.List[str]): keys this client has permission to write

    Args:
        name: the non-unique, but convenient identifier (stringifies the uuid if None) for the client
        unique_identifier: client's unique identifier (auto-generates if None)
        read: list of keys this client has permission to read
        write: list of keys this client has permission to write

    Raises:
        TypeError: if the provided name/unique identifier is not of type str/uuid.UUID
        ValueError: if the unique identifier has already been registered

    """
    storage = {}  # Dict[str, Any] / key-value storage
    metadata = {}  # Dict[ str, KeyMetaData ] / key-metadata information
    clients = {}   # Dict[ uuid.UUID, Blackboard] / id-client information
    activity_stream = None

    def __init__(
            self, *,
            name: str=None,
            unique_identifier: uuid.UUID=None,
            read: typing.Set[str]=set(),
            write: typing.Set[str]=set()):
        # print("__init__")
        if unique_identifier is None:
            unique_identifier = uuid.uuid4()
        if type(unique_identifier) != uuid.UUID:
            raise TypeError("provided unique identifier is not of type uuid.UUID")
        if name is None or not name:
            name = str(unique_identifier)
        if not isinstance(name, str):
            raise TypeError("provided name is not of type str [{}]".format(type(name)))
        if type(read) is list:
            read = set(read)
        if type(write) is list:
            write = set(write)
        if unique_identifier in Blackboard.clients.keys():
            raise ValueError("this unique identifier has already been registered")
        super().__setattr__("unique_identifier", unique_identifier)
        super().__setattr__("name", name)
        super().__setattr__("read", read)
        for key in read:
            Blackboard.metadata.setdefault(key, KeyMetaData())
            Blackboard.metadata[key].read.add(
                super().__getattribute__("unique_identifier")
            )
        super().__setattr__("write", write)
        for key in write:
            Blackboard.metadata.setdefault(key, KeyMetaData())
            Blackboard.metadata[key].write.add(
                super().__getattribute__("unique_identifier")
            )
        Blackboard.clients[
            super().__getattribute__("unique_identifier")
        ] = self

    def __setattr__(self, name, value):
        """
        Convenience attribute style referencing with checking against
        permissions.

        Raises:
            AttributeError: if the client does not have write access to the variable
        """
        # print("__setattr__ [{}][{}]".format(name, value))
        if name not in super().__getattribute__("write"):
            if Blackboard.activity_stream is not None:
                Blackboard.activity_stream.push(
                    self._generate_activity_item(name, ActivityType.WRITE_DENIED)
                )
            raise AttributeError("client '{}' does not have write access to '{}'".format(self.name, name))
        if Blackboard.activity_stream is not None:
            if name in Blackboard.storage.keys():
                Blackboard.activity_stream.push(
                    self._generate_activity_item(
                        key=name,
                        activity_type=ActivityType.WRITE,
                        previous_value=Blackboard.storage[name],
                        current_value=value
                    )
                )
            else:
                Blackboard.activity_stream.push(
                    self._generate_activity_item(
                        key=name,
                        activity_type=ActivityType.INITIALISED,
                        current_value=value
                    )
                )
        Blackboard.storage[name] = value

    def __getattr__(self, name):
        """
        Convenience attribute style referencing with checking against
        permissions.

        Raises:
            AttributeError: if the client does not have read access to the variable
            KeyError: if the variable does not yet exist on the blackboard
        """
        # print("__getattr__ [{}]".format(name))
        try:
            if name not in super().__getattribute__("read"):
                if Blackboard.activity_stream is not None:
                    Blackboard.activity_stream.push(
                        self._generate_activity_item(name, ActivityType.READ_DENIED)
                    )
                raise AttributeError("client '{}' does not have read access to '{}'".format(self.name, name))
            if Blackboard.activity_stream is not None:
                Blackboard.activity_stream.push(
                    self._generate_activity_item(
                        key=name,
                        activity_type=ActivityType.READ,
                        current_value=Blackboard.storage[name],
                    )
                )
            return Blackboard.storage[name]
        except KeyError as e:
            if Blackboard.activity_stream is not None:
                Blackboard.activity_stream.push(
                    self._generate_activity_item(name, ActivityType.READ_FAILED)
                )
            raise KeyError("variable '{}' does not yet exist on the blackboard".format(name)) from e

    def set(self, name: str, value: typing.Any, overwrite: bool=True) -> bool:
        """
        Set, conditionally depending on whether the variable already exists or otherwise.

        This is most useful when initialising variables and multiple elements
        seek to do so. A good policy to adopt for your applications in these situations is
        a first come, first served policy. Ensure global configuration has the first
        opportunity followed by higher priority behaviours in the tree and so forth.
        Lower priority behaviours would use this to respect the pre-configured
        setting and at most, just validate that it is acceptable to the functionality
        of it's own behaviour.

        Args:
            name: name of the variable to set
            value: value of the variable to set
            overwrite: do not set if the variable already exists on the blackboard

        Returns:
            success or failure (overwrite is False and variable already set)

        Raises:
            AttributeError: if the client does not have write access to the variable
        """
        if name not in super().__getattribute__("write"):
            if Blackboard.activity_stream is not None:
                Blackboard.activity_stream.push(
                    self._generate_activity_item(name, ActivityType.WRITE_DENIED)
                )
            raise AttributeError("client does not have write access to '{}'".format(name))
        if not overwrite:
            if name in Blackboard.storage:
                if Blackboard.activity_stream is not None:
                    Blackboard.activity_stream.push(
                        self._generate_activity_item(
                            key=name,
                            activity_type=ActivityType.NO_OVERWRITE,
                            current_value=Blackboard.storage[name])
                    )
                return False
        setattr(self, name, value)
        return True

    def get(self, name: str) -> typing.Any:
        """
        Method based accessor to the blackboard variables (as opposed to simply using
        '.<name>').

        Args:
            name: name of the variable to get

        Raises:
            AttributeError: if the client does not have read access to the variable
            KeyError: if the variable does not yet exist on the blackboard
        """
        return getattr(self, name)

    def unset(self, name: str):
        """
        For when you need to completely remove a blackboard variable,
        this provides a convenient helper method. This is particularly
        useful when you wish to 'erase' the blackboard between unit test
        methods.

        Args:
            name: name of the variable to clear

        Returns:
            True if the key , False otherwise
        """
        if Blackboard.activity_stream is not None:
            Blackboard.activity_stream.push(
                self._generate_activity_item(name, ActivityType.UNSET)
            )
        # Three means of handling a non-existent key - 1) raising a KeyError, 2) catching
        # the KeyError and passing, 3) catch the KeyError and return True/False.
        # Option 1) is inconvenient - requires a redundant try/catch 99% of cases
        # Option 2) hides information - bad
        # Option 3) no extra code necessary and information is there if desired
        try:
            del Blackboard.storage[name]
            return True
        except KeyError:
            return False

    def __str__(self):
        indent = "  "
        s = console.green + type(self).__name__ + console.reset + "\n"
        s += console.white + indent + "Client Data" + console.reset + "\n"
        keys = ["name", "unique_identifier", "read", "write"]
        s += self._stringify_key_value_pairs(keys, self.__dict__, 2 * indent)
        s += console.white + indent + "Variables" + console.reset + "\n"
        keys = self.read | self.write
        s += self._stringify_key_value_pairs(keys, Blackboard.storage, 2 * indent)
        return s

    def _generate_activity_item(self, key, activity_type, previous_value=None, current_value=None):
        return ActivityItem(
            key=key,
            client_name=super().__getattribute__("name"),
            client_id=super().__getattribute__("unique_identifier"),
            activity_type=activity_type,
            previous_value=previous_value,
            current_value=current_value
        )

    def _stringify_key_value_pairs(self, keys, key_value_dict, indent):
        s = ""
        max_length = 0
        for key in keys:
            max_length = len(key) if len(key) > max_length else max_length
        for key in keys:
            try:
                value = key_value_dict[key]
                lines = ('{0}'.format(value)).split('\n')
                if len(lines) > 1:
                    s += console.cyan + indent + '{0: <{1}}'.format(key, max_length + 1) + console.reset + ":\n"
                    for line in lines:
                        s += console.yellow + indent + "  {0}\n".format(line) + console.reset
                else:
                    s += console.cyan + indent + '{0: <{1}}'.format(key, max_length + 1) + console.reset + ": " + console.yellow + '{0}\n'.format(value) + console.reset
            except KeyError:
                s += console.cyan + indent + '{0: <{1}}'.format(key, max_length + 1) + console.reset + ": " + console.yellow + "-\n" + console.reset
        s += console.reset
        return s

    def unregister(self, clear=True):
        """
        Unregister this blackboard and if requested, clear key-value pairs if this
        client is the last user of those variables.
        """
        for key in self.read:
            Blackboard.metadata[key].read.remove(super().__getattribute__("unique_identifier"))
        for key in self.write:
            Blackboard.metadata[key].write.remove(super().__getattribute__("unique_identifier"))
        if clear:
            for key in (set(self.read) | set(self.write)):
                if not (set(Blackboard.metadata[key].read) | set(Blackboard.metadata[key].write)):
                    Blackboard.storage.pop(key, None)

    @staticmethod
    def keys() -> typing.Set[str]:
        """
        Get the set of blackboard keys.

        Returns:
            the complete set of keys registered by clients
        """
        # return registered keys, those on the blackboard are not
        # necessarily written to yet
        return Blackboard.metadata.keys()

    @staticmethod
    def keys_filtered_by_regex(regex: str) -> typing.Set[str]:
        """
        Get the set of blackboard keys filtered by regex.

        Args:
            regex: a python regex string

        Returns:
            subset of keys that have been registered and match the pattern
        """
        pattern = re.compile(regex)
        return [key for key in Blackboard.metadata.keys() if pattern.search(key) is not None]

    @staticmethod
    def keys_filtered_by_clients(client_ids: typing.Union[typing.List[str], typing.Set[str]]) -> typing.Set[str]:
        """
        Get the set of blackboard keys filtered by client ids.

        Args:
            client_ids: set of client uuid's.

        Returns:
            subset of keys that have been registered by the specified clients
        """
        # convenience for users
        if type(client_ids) == list:
            client_ids = set(client_ids)
        keys = set()
        for key in Blackboard.metadata.keys():
            # for sets, | is union, & is intersection
            key_clients = set(Blackboard.metadata[key].read) | set(Blackboard.metadata[key].write)
            if key_clients & client_ids:
                keys.add(key)
        return keys

    @staticmethod
    def enable_activity_stream(maximum_size: int=500):
        """
        Enable logging of activities on the blackboard.

        Args:
            maximum_size: pop items from the stream if this size is exceeded

        Raises:
            RuntimeError if the activity stream is already enabled
        """
        if Blackboard.activity_stream is None:
            Blackboard.activity_stream = ActivityStream(maximum_size)
        else:
            RuntimeError("activity stream is already enabled for this blackboard")

    @staticmethod
    def disable_activity_stream():
        """
        Disable logging of activities on the blackboard
        """
        Blackboard.activity_stream = None

##############################################################################
# Blackboard Behaviours
##############################################################################


class ClearBlackboardVariable(behaviours.Success):
    """
    Clear the specified value from the blackboard.

    Args:
        name (:obj:`str`): name of the behaviour
        variable_name (:obj:`str`): name of the variable to clear
    """
    def __init__(self,
                 name="Clear Blackboard Variable",
                 variable_name="dummy",
                 ):
        super(ClearBlackboardVariable, self).__init__(name)
        self.variable_name = variable_name
        self.blackboard = Blackboard(
            name=self.name,
            write={self.variable_name}
        )

    def initialise(self):
        """
        Delete the variable from the blackboard.
        """
        self.blackboard.unset(self.variable_name)


class SetBlackboardVariable(behaviours.Success):
    """
    Set the specified variable on the blackboard.
    Usually we set variables from inside other behaviours, but can
    be convenient to set them from a behaviour of their own sometimes so you
    don't get blackboard logic mixed up with more atomic behaviours.

    Args:
        name (:obj:`str`): name of the behaviour
        variable_name (:obj:`str`): name of the variable to set
        variable_value (:obj:`any`): value of the variable to set

    .. todo:: overwrite option, leading to possible failure/success logic.
    """
    def __init__(self,
                 name="Set Blackboard Variable",
                 variable_name="dummy",
                 variable_value=None
                 ):
        """
        :param name: name of the behaviour
        :param variable_name: name of the variable to set
        :param value_name: value of the variable to set
        """
        super(SetBlackboardVariable, self).__init__(name)
        self.variable_name = variable_name
        self.variable_value = variable_value
        self.blackboard = Blackboard(
            name=self.name,
            unique_identifier=self.id,
            write={self.variable_name}
        )

    def initialise(self):
        self.blackboard.set(self.variable_name, self.variable_value, overwrite=True)


class CheckBlackboardVariable(behaviour.Behaviour):
    """
    Check the blackboard to see if it has a specific variable
    and optionally whether that variable has an expected value.
    It is a binary behaviour, always updating it's status
    with either :data:`~py_trees.common.Status.SUCCESS` or
    :data:`~py_trees.common.Status.FAILURE` at each tick.
    """
    def __init__(self,
                 name,
                 variable_name="dummy",
                 expected_value=None,
                 comparison_operator=operator.eq,
                 clearing_policy=common.ClearingPolicy.ON_INITIALISE,
                 debug_feedback_message=False
                 ):
        """
        Initialise the behaviour. It's worth noting that there are a few
        combinations to the configuration that serve different use cases.

        Args:
            name (:obj:`str`): name of the behaviour
            variable_name (:obj:`str`): name of the variable to set
            expected_value (:obj:`any`): expected value to find (if `None`, check for existence only)
            comparison_operator (:obj:`func`): one from the python `operator module`_
            clearing_policy (:obj:`any`): when to clear the match result, see :py:class:`~py_trees.common.ClearingPolicy`
            debug_feedback_message (:obj:`bool`): provide additional detail in behaviour feedback messages for debugging

        .. tip::
            If just checking for existence, use the default argument
            on construction, `expected_value=None`.

        .. tip::
            There are times when you want to get the expected match once and then save
            that result thereafter. For example, to flag once a system has reached a
            subgoal. Use the :data:`~py_trees.common.ClearingPolicy.NEVER` flag to do this.
        """
        super(CheckBlackboardVariable, self).__init__(name)

        name_components = variable_name.split('.')
        self.variable_name = name_components[0]
        self.nested_name = '.'.join(name_components[1:])  # empty string if no other parts

        self.blackboard = Blackboard(
            name=self.name,
            unique_identifier=self.id,
            read={self.variable_name}
        )
        self.expected_value = expected_value
        self.comparison_operator = comparison_operator
        self.matching_result = None
        self.clearing_policy = clearing_policy
        self.debug_feedback_message = debug_feedback_message

    def initialise(self):
        """
        Clears the internally stored message ready for a new run
        if ``old_data_is_valid`` wasn't set.
        """
        self.logger.debug("%s.initialise()" % self.__class__.__name__)
        if self.clearing_policy == common.ClearingPolicy.ON_INITIALISE:
            self.matching_result = None

    def update(self):
        """
        Check for existence, or the appropriate match on the expected value.

        Returns:
             :class:`~py_trees.common.Status`: :data:`~py_trees.common.Status.FAILURE` if not matched, :data:`~py_trees.common.Status.SUCCESS` otherwise.
        """
        self.logger.debug("%s.update()" % self.__class__.__name__)
        if self.matching_result is not None:
            return self.matching_result

        result = None

        try:
            # value = check_attr(self.blackboard)
            value = self.blackboard.get(self.variable_name)
            if self.nested_name:
                try:
                    value = operator.attrgetter(self.nested_name)(value)
                except AttributeError:
                    raise KeyError()
            # if existence check required only
            if self.expected_value is None:
                self.feedback_message = "'%s' exists on the blackboard (as required)" % self.variable_name
                result = common.Status.SUCCESS
        except KeyError:
            name = "{}.{}".format(self.variable_name, self.nested_name) if self.nested_name else self.variable_name
            self.feedback_message = 'blackboard variable {0} did not exist'.format(name)
            result = common.Status.FAILURE

        if result is None:
            # expected value matching
            # value = getattr(self.blackboard, self.variable_name)
            success = self.comparison_operator(value, self.expected_value)

            if success:
                if self.debug_feedback_message:  # costly
                    self.feedback_message = "'%s' comparison succeeded [v: %s][e: %s]" % (self.variable_name, value, self.expected_value)
                else:
                    self.feedback_message = "'%s' comparison succeeded" % (self.variable_name)
                result = common.Status.SUCCESS
            else:
                if self.debug_feedback_message:  # costly
                    self.feedback_message = "'%s' comparison failed [v: %s][e: %s]" % (self.variable_name, value, self.expected_value)
                else:
                    self.feedback_message = "'%s' comparison failed" % (self.variable_name)
                result = common.Status.FAILURE

        if result == common.Status.SUCCESS and self.clearing_policy == common.ClearingPolicy.ON_SUCCESS:
            self.matching_result = None
        else:
            self.matching_result = result
        return result

    def terminate(self, new_status):
        """
        Always discard the matching result if it was invalidated by a parent or
        higher priority interrupt.
        """
        self.logger.debug("%s.terminate(%s)" % (self.__class__.__name__, "%s->%s" % (self.status, new_status) if self.status != new_status else "%s" % new_status))
        if new_status == common.Status.INVALID:
            self.matching_result = None


class WaitForBlackboardVariable(behaviour.Behaviour):
    """
    Check the blackboard to see if it has a specific variable
    and optionally whether that variable has a specific value.
    Unlike :py:class:`~py_trees.blackboard.CheckBlackboardVariable`
    this class will be in a :data:`~py_trees.common.Status.RUNNING` state until the variable appears
    and (optionally) is matched.

    Args:
        name (:obj:`str`): name of the behaviour
        variable_name (:obj:`str`): name of the variable to check
        expected_value (:obj:`any`): expected value to find (if `None`, check for existence only)
        comparison_operator (:obj:`func`): one from the python `operator module`_
        clearing_policy (:obj:`any`): when to clear the match result, see :py:class:`~py_trees.common.ClearingPolicy`

    .. tip::
        There are times when you want to get the expected match once and then save
        that result thereafter. For example, to flag once a system has reached a
        subgoal. Use the :data:`~py_trees.common.ClearingPolicy.NEVER` flag to do this.

    .. seealso:: :class:`~py_trees.blackboard.CheckBlackboardVariable`

    .. include:: weblinks.rst
    """
    def __init__(self,
                 name,
                 variable_name="dummy",
                 expected_value=None,
                 comparison_operator=operator.eq,
                 clearing_policy=common.ClearingPolicy.ON_INITIALISE
                 ):
        super(WaitForBlackboardVariable, self).__init__(name)
        name_components = variable_name.split('.')
        self.variable_name = name_components[0]
        self.nested_name = '.'.join(name_components[1:])  # empty string if no other parts
        self.blackboard = Blackboard(
            name=self.name,
            unique_identifier=self.id,
            read={self.variable_name}
        )
        self.expected_value = expected_value
        self.comparison_operator = comparison_operator
        self.clearing_policy = clearing_policy
        self.matching_result = None

    def initialise(self):
        """
        Clears the internally stored message ready for a new run
        if ``old_data_is_valid`` wasn't set.
        """
        self.logger.debug("%s.initialise()" % self.__class__.__name__)
        if self.clearing_policy == common.ClearingPolicy.ON_INITIALISE:
            self.matching_result = None
        self.check_attr = operator.attrgetter(self.variable_name)

    def update(self):
        """
        Check for existence, or the appropriate match on the expected value.

        Returns:
             :class:`~py_trees.common.Status`: :data:`~py_trees.common.Status.FAILURE` if not matched, :data:`~py_trees.common.Status.SUCCESS` otherwise.
        """
        self.logger.debug("%s.update()" % self.__class__.__name__)
        if self.matching_result is not None:
            return self.matching_result

        # existence failure check
        try:
            value = self.blackboard.get(self.variable_name)
            if self.nested_name:
                try:
                    value = operator.attrgetter(self.nested_name)(value)
                except AttributeError:
                    raise KeyError()  # type raised when no variable exists, caught below
            # if existence check required only
            if self.expected_value is None:
                self.feedback_message = "'%s' exists on the blackboard (as required)" % self.variable_name
                result = common.Status.SUCCESS
            # expected value matching
            else:
                success = self.comparison_operator(value, self.expected_value)
                if success:
                    self.feedback_message = "'%s' comparison succeeded [v: %s][e: %s]" % (self.variable_name, value, self.expected_value)
                    result = common.Status.SUCCESS
                else:
                    self.feedback_message = "'%s' comparison failed [v: %s][e: %s]" % (self.variable_name, value, self.expected_value)
                    result = common.Status.RUNNING
        except KeyError:
            name = "{}.{}".format(self.variable_name, self.nested_name) if self.nested_name else self.variable_name
            self.feedback_message = 'variable {0} did not exist'.format(name)
            result = common.Status.RUNNING

        if result == common.Status.SUCCESS and self.clearing_policy == common.ClearingPolicy.ON_SUCCESS:
            self.matching_result = None
        elif result != common.Status.RUNNING:  # will fall in here if clearing ON_INITIALISE, or NEVER
            self.matching_result = result
        return result

    def terminate(self, new_status):
        """
        Always discard the matching result if it was invalidated by a parent or
        higher priority interrupt.
        """
        self.logger.debug("%s.terminate(%s)" % (self.__class__.__name__, "%s->%s" % (self.status, new_status) if self.status != new_status else "%s" % new_status))
        if new_status == common.Status.INVALID:
            self.matching_result = None


##############################################################################
# Main
##############################################################################


def main():
    bb2 = Blackboard(
        name="Bob's Board",
        unique_identifier=uuid.uuid4(),
        read=['foo', 'bar'],
        write=['foo', 'dude'],
    )
    bb2.foo = "more bar"
    bb2.dude = "bob"
    unused_myfoo = bb2.foo
    bb2.introspect_blackboard()
    print("-----------------")
    print("{}".format(str(bb2)))
    print("-----------------")
    print(console.green + "Exceptions\n" + console.reset)
    try:
        print(console.green + "  get(<key>): doesn't exist on the blackboard" + console.reset)
        print("bar: {}".format(bb2.get('bar')))
    except Exception as e:
        print("    {}: {}".format(type(e), str(e)))
    try:
        print(console.green + "  get(<key>): doesn't have read access..." + console.reset)
        print("dude: {}".format(bb2.get('dude')))
    except Exception as e:
        print("    {}: {}".format(type(e), str(e)))
    try:
        print(console.green + "  .<key>: doesn't exist on the blackboard" + console.reset)
        print("bar: {}".format(bb2.bar))
    except Exception as e:
        print("    {}: {}".format(type(e), str(e)))
    try:
        print(console.green + "  .<key>: doesn't have read access..." + console.reset)
        print("dude: {}".format(bb2.dude))
    except Exception as e:
        print("    {}: {}".format(type(e), str(e)))
    # set
    try:
        print(console.green + "  set(<key>): doesn't have write permissions" + console.reset)
        bb2.set('foobar', 3)
    except Exception as e:
        print("    {}: {}".format(type(e), str(e)))
    try:
        print(console.green + "  set(<key>): could not overwrite existing variable" + console.reset)
        bb2.set('foo', 3, overwrite=False)
    except Exception as e:
        print("    {}: {}".format(type(e), str(e)))
    try:
        print(console.green + "  .<key>=...: doesn't have write permissions" + console.reset)
        bb2.foobar = 3
    except Exception as e:
        print("    {}: {}".format(type(e), str(e)))

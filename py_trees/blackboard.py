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

import copy
import enum
import re
import typing
import uuid

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
    """Initialised a key-value pair on the blackboard"""
    WRITE = "WRITE"
    """Wrote to the blackboard."""
    ACCESSED = "ACCESSED"
    """Key accessed, either for reading, or modification of the value's internal attributes (e.g. foo.bar)."""
    ACCESS_DENIED = "ACCESS_DENIED"
    """Client did not have access to read/write a key."""
    NO_KEY = "NO_KEY"
    """Tried to access a key that does not yet exist on the blackboard."""
    NO_OVERWRITE = "NO_OVERWRITE"
    """Tried to write but variable already exists and a no-overwrite request was respected."""
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
                    self._generate_activity_item(name, ActivityType.ACCESS_DENIED)
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
        if name not in (super().__getattribute__("read") | super().__getattribute__("write")):
            if Blackboard.activity_stream is not None:
                Blackboard.activity_stream.push(
                    self._generate_activity_item(name, ActivityType.ACCESS_DENIED)
                )
            raise AttributeError("client '{}' does not have read/write access to '{}'".format(self.name, name))
        try:
            if name in super().__getattribute__("write"):
                if Blackboard.activity_stream is not None:
                    Blackboard.activity_stream.push(
                        self._generate_activity_item(
                            key=name,
                            activity_type=ActivityType.ACCESSED,
                            current_value=Blackboard.storage[name],
                        )
                    )
                return Blackboard.storage[name]
            if name in super().__getattribute__("read"):
                if Blackboard.activity_stream is not None:
                    Blackboard.activity_stream.push(
                        self._generate_activity_item(
                            key=name,
                            activity_type=ActivityType.ACCESSED,
                            current_value=Blackboard.storage[name],
                        )
                    )
                return copy.deepcopy(Blackboard.storage[name])
        except KeyError as e:
            if Blackboard.activity_stream is not None:
                Blackboard.activity_stream.push(
                    self._generate_activity_item(name, ActivityType.NO_KEY)
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
                    self._generate_activity_item(name, ActivityType.ACCESS_DENIED)
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

    def unset(self, key: str):
        """
        For when you need to completely remove a blackboard variable (key-value pair),
        this provides a convenient helper method.

        Args:
            key: name of the variable to remove

        Returns:
            True if the variable was removed, False if it was already absent
        """
        if Blackboard.activity_stream is not None:
            Blackboard.activity_stream.push(
                self._generate_activity_item(key, ActivityType.UNSET)
            )
        # Three means of handling a non-existent key - 1) raising a KeyError, 2) catching
        # the KeyError and passing, 3) catch the KeyError and return True/False.
        # Option 1) is inconvenient - requires a redundant try/catch 99% of cases
        # Option 2) hides information - bad
        # Option 3) no extra code necessary and information is there if desired
        try:
            del Blackboard.storage[key]
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


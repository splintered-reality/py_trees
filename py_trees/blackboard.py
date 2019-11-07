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
* [+] Client style usage with registration of read/write access
* [+] Activity stream that logs read/write operations by clients
* [+] Integration with behaviors for key-behaviour associations (debugging)
* [-] Exclusive locks for writing
* [-] Sharing between tree instances
* [-] Priority policies for variable instantiations

.. include:: weblinks.rst
"""

##############################################################################
# Imports
##############################################################################

import enum
import operator
import re
import typing
import uuid

from . import common
from . import console
from . import utilities

##############################################################################
# Classes
##############################################################################


class KeyMetaData(object):
    """
    Stores the aggregated metadata for a key on the blackboard.
    """
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
    Centralised key-value store for sharing data between behaviours.
    This class is a coat-hanger for the centralised data store, metadata
    for it's administration and static methods for interacting with it.

    This api is intended for authors of debugging and introspection
    tools on the blackboard. Users should make use of the :class:`Client`.

    Attributes:
        Blackboard.clients (typing.Dict[uuid.UUID, Blackboard]): clients, gathered by uuid
        Blackboard.storage (typing.Dict[str, typing.Any]): key-value data store
        Blackboard.metadata (typing.Dict[str, KeyMetaData]): key associated metadata
        Blackboard.activity_stream (ActivityStream): logged activity
    """
    storage = {}  # Dict[str, Any] / key-value storage
    metadata = {}  # Dict[ str, KeyMetaData ] / key-metadata information
    clients = {}   # Dict[ uuid.UUID, Blackboard] / id-client information
    activity_stream = None

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
    def get(variable_name: str) -> typing.Any:
        """
        Extract the value associated with the given a variable name,
        can be nested, e.g. battery.percentage. This differs from the
        client get method in that it doesn't pass through the client access
        checks. To be used for utility tooling (e.g. display methods) and not by
        users directly.

        Args:
            variable_name: of the variable to get, can be nested, e.g. battery.percentage

        Raises:
            KeyError: if the variable or it's nested attributes do not yet exist on the blackboard

        Return:
            The stored value for the given variable
        """
        # convenience, just in case they used slashes instead of .'s
        if '/' in variable_name:
            variable_name = ".".join(variable_name.split('/'))
        name_components = variable_name.split('.')
        key = name_components[0]
        key_attributes = '.'.join(name_components[1:])
        # can raise KeyError
        value = Blackboard.storage[key]
        if key_attributes:
            try:
                value = operator.attrgetter(key_attributes)(value)
            except AttributeError:
                raise KeyError("Key exists, but does not have the specified nested attributes [{}]".format(variable_name))
        return value

    @staticmethod
    def set(variable_name: str, value: typing.Any):
        """
        Set the value associated with the given a variable name,
        can be nested, e.g. battery.percentage. This differs from the
        client get method in that it doesn't pass through the client access
        checks. To be used for utility tooling (e.g. display methods) and not by
        users directly.

        Args:
            variable_name: of the variable to set, can be nested, e.g. battery.percentage

        Raises:
            AttributeError: if it is attempting to set a nested attribute tha does not exist.
        """
        name_components = variable_name.split('.')
        key = name_components[0]
        key_attributes = '.'.join(name_components[1:])
        if not key_attributes:
            Blackboard.storage[key] = value
        else:
            setattr(Blackboard.storage[key], key_attributes, value)

    @staticmethod
    def unset(key: str):
        """
        For when you need to completely remove a blackboard variable (key-value pair),
        this provides a convenient helper method.

        Args:
            key: name of the variable to remove

        Returns:
            True if the variable was removed, False if it was already absent
        """
        try:
            del Blackboard.storage[key]
            return True
        except KeyError:
            return False

    @staticmethod
    def exists(name: str) -> bool:
        """
        Check if the specified variable exists on the blackboard.

        Args:
            name: name of the variable, can be nested, e.g. battery.percentage

        Raises:
            AttributeError: if the client does not have read access to the variable
        """
        try:
            unused_value = Blackboard.get(name)
            return True
        except KeyError:
            return False

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

    @staticmethod
    def clear():
        """
        Completely clear all key, value and client information from the blackboard.
        Also deletes the activity stream.
        """
        Blackboard.storage.clear()
        Blackboard.metadata.clear()
        Blackboard.clients.clear()
        Blackboard.activity_stream = None


class Client(object):
    """
    Client to the key-value store for sharing data between behaviours.

    **Examples**

    Blackboard clients will accept a user-friendly name / unique identifier for
    registration on the centralised store or create them for you if none is provided.

    .. code-block:: python

        provided = py_trees.blackboard.Client(name="Provided")
        print(provided)
        generated = py_trees.blackboard.Client()
        print(generated)

    .. figure:: images/blackboard_client_instantiation.png
       :align: center

       Client Instantiation

    Register read/write access for keys on the blackboard. Note, registration is
    not initialisation.

    .. code-block:: python

        blackboard = py_trees.blackboard.Client(name="Client")
        blackboard.register_key(key="foo", access=py_trees.common.Access.WRITE)
        blackboard.register_key(key="bar", access=py_trees.common.Access.READ)
        blackboard.foo = "foo"
        print(blackboard)

    .. figure:: images/blackboard_read_write.png
       :align: center

       Variable Read/Write Registration

    Disconnected instances will discover the centralised
    key-value store.

    .. code-block:: python

        def check_foo():
            blackboard = py_trees.blackboard.Client(name="Reader")
            blackboard.register_key(key="foo", access=py_trees.common.Access.READ)
            print("Foo: {}".format(blackboard.foo))


        blackboard = py_trees.blackboard.Client(name="Writer")
        blackboard.register_key(key="foo", access=py_trees.common.Access.WRITE)
        blackboard.foo = "bar"
        check_foo()

    To respect an already initialised key on the blackboard:

    .. code-block:: python

        blackboard = Client(name="Writer")
        blackboard.register_key(key="foo", access=py_trees.common.Access.READ)
        result = blackboard.set("foo", "bar", overwrite=False)

    Store complex objects on the blackboard:

    .. code-block:: python

        class Nested(object):
            def __init__(self):
                self.foo = None
                self.bar = None

            def __str__(self):
                return str(self.__dict__)


        writer = py_trees.blackboard.Client(name="Writer")
        writer.register_key(key="nested", access=py_trees.common.Access.WRITE)
        reader = py_trees.blackboard.Client(name="Reader")
        reader.register_key(key="nested", access=py_trees.common.Access.READ)

        writer.nested = Nested()
        writer.nested.foo = "I am foo"
        writer.nested.bar = "I am bar"

        foo = reader.nested.foo
        print(writer)
        print(reader)

    .. figure:: images/blackboard_nested.png
       :align: center

    Log and display the activity stream:

    .. code-block:: python

        py_trees.blackboard.Blackboard.enable_activity_stream(maximum_size=100)
        reader = py_trees.blackboard.Client(name="Reader")
        reader.register_key(key="foo", access=py_trees.common.Access.READ)
        writer = py_trees.blackboard.Client(name="Writer")
        writer.register_key(key="foo", access=py_trees.common.Access.WRITE)
        writer.foo = "bar"
        writer.foo = "foobar"
        unused_result = reader.foo
        print(py_trees.display.unicode_blackboard_activity_stream())
        py_trees.blackboard.Blackboard.activity_stream.clear()

    .. figure:: images/blackboard_activity_stream.png
       :align: center

    Display the blackboard on the console, or part thereof:

    .. code-block:: python

        writer = py_trees.blackboard.Client(name="Writer")
        for key in {"foo", "bar", "dude", "dudette"}:
            writer.register_key(key=key, access=py_trees.common.Access.WRITE)

        reader = py_trees.blackboard.Client(name="Reader")
        for key in {"foo", "bar"}:
            reader.register_key(key="key", access=py_trees.common.Access.READ)

        writer.foo = "foo"
        writer.bar = "bar"
        writer.dude = "bob"

        # all key-value pairs
        print(py_trees.display.unicode_blackboard())
        # various filtered views
        print(py_trees.display.unicode_blackboard(key_filter={"foo"}))
        print(py_trees.display.unicode_blackboard(regex_filter="dud*"))
        print(py_trees.display.unicode_blackboard(client_filter={reader.unique_identifier}))
        # list the clients associated with each key
        print(py_trees.display.unicode_blackboard(display_only_key_metadata=True))

    .. figure:: images/blackboard_display.png
       :align: center

    Behaviours are not automagically connected to the blackboard but you may
    manually attach one or more clients so that associations between behaviours
    and variables can be tracked - this is very useful for introspection and
    debugging.

    Creating a custom behaviour with blackboard variables:

    .. code-block:: python

        class Foo(py_trees.behaviour.Behaviour):

            def __init__(self, name):
                super().__init__(name=name)
                self.blackboard = self.attach_blackboard_client(name="Foo Global")
                self.parameters = self.attach_blackboard_client(name="Foo Params", namespace="foo_parameters_")
                self.state = self.attach_blackboard_client(name="Foo State", namespace="foo_state_")

                # create a key 'foo_parameters_init' on the blackboard
                self.parameters.register_key("init", access=py_trees.common.Access.READ)
                # create a key 'foo_state_number_of_noodles' on the blackboard
                self.state.register_key("number_of_noodles", access=py_trees.common.Access.WRITE)

            def initialise(self):
                self.state.number_of_noodles = self.parameters.init

            def update(self):
                self.state.number_of_noodles += 1
                self.feedback_message = self.state.number_of_noodles
                if self.state.number_of_noodles > 5:
                    return py_trees.common.Status.SUCCESS
                else:
                    return py_trees.common.Status.RUNNING


        # could equivalently do directly via the Blackboard static methods if
        # not interested in tracking / visualising the application configuration
        configuration = py_trees.blackboard.Client(name="App Config")
        configuration.register_key("foo_parameters_init", access=py_trees.common.Access.WRITE)
        configuration.foo_parameters_init = 3

        foo = Foo(name="The Foo")
        for i in range(1, 8):
            foo.tick_once()
            print("Number of Noodles: {}".format(foo.feedback_message))

    Rendering a dot graph for a behaviour tree, complete with blackboard variables:

    .. code-block:: python

        # in code
        py_trees.display.render_dot_tree(py_trees.demos.blackboard.create_root())
        # command line tools
        py-trees-render --with-blackboard-variables py_trees.demos.blackboard.create_root

    .. graphviz:: dot/demo-blackboard.dot
       :align: center
       :caption: Tree with Blackboard Variables

    And to demonstrate that it doesn't become a tangled nightmare at scale, an example of
    a more complex tree:

    .. graphviz:: dot/blackboard-with-variables.dot
       :align: center
       :caption: A more complex tree

    Debug deeper with judicious application of the tree, blackboard and activity stream
    display methods around the tree tick (refer to
    :class:`py_trees.visitors.DisplaySnapshotVisitor` for examplar code):

    .. figure:: images/blackboard_trees.png
       :align: center

       Tree level debugging

    .. seealso::

       * :ref:`py-trees-demo-blackboard <py-trees-demo-blackboard-program>`
       * :class:`py_trees.visitors.DisplaySnapshotVisitor`
       * :class:`py_trees.behaviours.SetBlackboardVariable`
       * :class:`py_trees.behaviours.UnsetBlackboardVariable`
       * :class:`py_trees.behaviours.CheckBlackboardVariableExists`
       * :class:`py_trees.behaviours.WaitForBlackboardVariable`
       * :class:`py_trees.behaviours.CheckBlackboardVariableValue`
       * :class:`py_trees.behaviours.WaitForBlackboardVariableValue`

    Attributes:
        name (str): client's convenient, but not necessarily unique identifier
        namespace (str): apply this as a prefix to any key/variable name operations
        unique_identifier (uuid.UUID): client's unique identifier
    """
    def __init__(
            self, *,
            name: str=None,
            namespace: str=None):
        """
        Args:
            name: client's convenient identifier (stringifies the uuid if None)
            namespace: prefix to apply to key/variable name operations
            read: list of keys for which this client has read access
            write: list of keys for which this client has write access

        Raises:
            TypeError: if the provided name is not of type str
            ValueError: if the unique identifier has already been registered
        """

        # unique identifier
        super().__setattr__("unique_identifier", uuid.uuid4())
        if super().__getattribute__("unique_identifier") in Blackboard.clients.keys():
            raise ValueError("this unique identifier has already been registered")

        # name
        if name is None or not name:
            name = utilities.truncate(
                original=str(super().__getattribute__("unique_identifier")).replace('-', '_'),
                length=7
            )
            super().__setattr__("name", name)
        else:
            if not isinstance(name, str):
                raise TypeError("provided name is not of type str [{}]".format(type(name)))
            super().__setattr__("name", name)

        # namespace
        namespace = "" if namespace is None else namespace
        super().__setattr__("namespace", namespace)

        super().__setattr__("read", set())
        super().__setattr__("write", set())
        super().__setattr__("required", set())
        Blackboard.clients[
            super().__getattribute__("unique_identifier")
        ] = self

    def id(self) -> uuid.UUID:
        """
        The unique identifier for this client.

        Returns:
            The uuid.UUID object
        """
        return super().__getattribute__("unique_identifier")

    def __setattr__(self, name: str, value: typing.Any):
        """
        Convenience attribute style referencing with checking against
        permissions.

        Raises:
            AttributeError: if the client does not have write access to the variable
        """
        # print("__setattr__ [{}][{}]".format(name, value))
        local_name = name
        name = super().__getattribute__("namespace") + name
        if local_name not in super().__getattribute__("write"):
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

    def __getattr__(self, name: str):
        """
        Convenience attribute style referencing with checking against
        permissions.

        Raises:
            AttributeError: if the client does not have read access to the variable
            KeyError: if the variable does not yet exist on the blackboard
        """
        # print("__getattr__ [{}]".format(name))
        local_name = name
        name = super().__getattribute__("namespace") + name
        if local_name not in (super().__getattribute__("read") | super().__getattribute__("write")):
            if Blackboard.activity_stream is not None:
                Blackboard.activity_stream.push(
                    self._generate_activity_item(name, ActivityType.ACCESS_DENIED)
                )
            raise AttributeError("client '{}' does not have read/write access to '{}'".format(self.name, name))
        try:
            if local_name in super().__getattribute__("write"):
                if Blackboard.activity_stream is not None:
                    Blackboard.activity_stream.push(
                        self._generate_activity_item(
                            key=name,
                            activity_type=ActivityType.ACCESSED,
                            current_value=Blackboard.storage[name],
                        )
                    )
                return Blackboard.storage[name]
            if local_name in super().__getattribute__("read"):
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
                    self._generate_activity_item(name, ActivityType.NO_KEY)
                )
            raise KeyError("client '{}' tried to access '{}' but it does not yet exist on the blackboard".format(self.name, name)) from e

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
            KeyError: if the variable does not yet exist on the blackboard
        """
        name_components = name.split('.')
        key = name_components[0]
        namespaced_key = super().__getattribute__("namespace") + key
        key_attributes = '.'.join(name_components[1:])
        if key not in super().__getattribute__("write"):
            if Blackboard.activity_stream is not None:
                Blackboard.activity_stream.push(
                    self._generate_activity_item(name, ActivityType.ACCESS_DENIED)
                )
            raise AttributeError("client '{}' does not have write access to '{}'".format(self.name, name))
        if not overwrite:
            if namespaced_key in Blackboard.storage:
                if Blackboard.activity_stream is not None:
                    Blackboard.activity_stream.push(
                        self._generate_activity_item(
                            key=key,
                            activity_type=ActivityType.NO_OVERWRITE,
                            current_value=Blackboard.storage[namespaced_key])
                    )
                return False
        if not key_attributes:
            setattr(self, name, value)
            return True
        else:
            blackboard_object = getattr(self, key)
            try:
                setattr(blackboard_object, key_attributes, value)
                return True
            except AttributeError:  # when the object doesn't have the attributes
                return False

    def exists(self, name: str) -> bool:
        """
        Check if the specified variable exists on the blackboard.

        Args:
            name: name of the variable to get, can be nested, e.g. battery.percentage

        Raises:
            AttributeError: if the client does not have read access to the variable
        """
        try:
            unused_value = self.get(name)
            return True
        except KeyError:
            return False

    def get(self, name: str) -> typing.Any:
        """
        Method based accessor to the blackboard variables (as opposed to simply using
        '.<name>').

        Args:
            name: name of the variable to get, can be nested, e.g. battery.percentage

        Raises:
            AttributeError: if the client does not have read access to the variable
            KeyError: if the variable or it's nested attributes do not yet exist on the blackboard
        """
        # key attributes is an empty string if not a nested variable name
        name_components = name.split('.')
        key = name_components[0]
        key_attributes = '.'.join(name_components[1:])
        value = getattr(self, key)  # will run through client access checks in __getattr__
        if key_attributes:
            try:
                value = operator.attrgetter(key_attributes)(value)
            except AttributeError:
                raise KeyError("Key exists, but does not have the specified nested attributes [{}]".format(name))
        return value

    def unset(self, key: str):
        """
        For when you need to completely remove a blackboard variable (key-value pair),
        this provides a convenient helper method.

        Args:
            key: name of the variable to remove

        Returns:
            True if the variable was removed, False if it was already absent
        """
        key = super().__getattribute__("namespace") + key
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
        s = console.green + "Blackboard Client" + console.reset + "\n"
        s += console.white + indent + "Client Data" + console.reset + "\n"
        keys = ["name", "namespace", "unique_identifier", "read", "write"]
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

    def unregister(self, clear: bool=True):
        """
        Unregister this blackboard client and if requested, clear key-value pairs if this
        client is the last user of those variables.

        Args:
            clear: remove key-values pairs from the blackboard
        """
        self.unregister_all_keys(clear)
        del Blackboard.clients[super().__getattribute__("unique_identifier")]

    def unregister_all_keys(self, clear: bool=True):
        """
        Unregister all keys currently registered by this blackboard client and if requested,
        clear key-value pairs if this client is the last user of those variables.

        Args:
            clear: remove key-values pairs from the blackboard
        """
        for local_key in self.read:
            key = super().__getattribute__("namespace") + local_key
            Blackboard.metadata[key].read.remove(super().__getattribute__("unique_identifier"))
        for local_key in self.write:
            key = super().__getattribute__("namespace") + local_key
            Blackboard.metadata[key].write.remove(super().__getattribute__("unique_identifier"))
        if clear:
            for local_key in (set(self.read) | set(self.write)):
                key = super().__getattribute__("namespace") + local_key
                if not (set(Blackboard.metadata[key].read) | set(Blackboard.metadata[key].write)):
                    try:
                        del Blackboard.storage[key]
                    except KeyError:
                        pass  # perfectly acceptal for a key to not exist on the blackboard yet
                    del Blackboard.metadata[key]

    def verify_required_keys_exist(self):
        """
        En-masse check of existence on the blackboard for all keys that were hitherto
        registered as 'required'.

        Raises: KeyError if any of the required keys do not exist on the blackboard
        """
        absent = set()
        for local_key in super().__getattribute__("required"):
            if not self.exists(local_key):
                absent.add(local_key)
        if absent:
            raise KeyError("keys required, but not yet on the blackboard [{}]".format(absent))

    def register_key(
            self,
            key: str,
            access: common.Access,
            required: bool=False
    ):
        """
        Register a key on the blackboard to associate with this client.

        Args:
            key: key to register
            access: access level (read, write, exclusive write)
            required: if true, check key exists when calling
                :meth:`~verify_required_keys_exist`

        Raises:
            TypeError if the access argument is of incorrect type
        """
        local_key = key
        key = super().__getattribute__("namespace") + key
        Blackboard.metadata.setdefault(key, KeyMetaData())
        if access == common.Access.READ:
            super().__getattribute__("read").add(local_key)
            Blackboard.metadata[key].read.add(super().__getattribute__("unique_identifier"))
        elif access == common.Access.WRITE:
            super().__getattribute__("write").add(local_key)
            Blackboard.metadata[key].write.add(super().__getattribute__("unique_identifier"))
        else:
            raise TypeError("access argument is of incorrect type [{}]".format(type(access)))
        if required:
            super().__getattribute__("required").add(local_key)

    def unregister_key(self, key: str, clear: bool=True):
        """
        Unegister a key associated with this client.

        Args:
            key: key to unregister
            clear: remove key-values pairs from the blackboard

        Raises:
            KeyError if the key has not been previously registered
        """
        local_key = key
        key = super().__getattribute__("namespace") + key
        super().__getattribute__("read").discard(local_key)  # doesn't throw exceptions if it not present
        super().__getattribute__("write").discard(local_key)
        Blackboard.metadata[key].read.discard(super().__getattribute__("unique_identifier"))
        Blackboard.metadata[key].write.discard(super().__getattribute__("unique_identifier"))
        if not (Blackboard.metadata[key].read | Blackboard.metadata[key].write):
            del Blackboard.metadata[key]
            if clear:
                try:
                    del Blackboard.storage[key]
                except KeyError:
                    pass  # perfectly legitimate for a registered key to not exist on the blackboard

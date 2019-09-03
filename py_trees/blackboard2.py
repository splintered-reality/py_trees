#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import enum
import typing
import uuid

from . import console
from gi.overrides.Gdk import name


class KeyMetaData(object):

    def __init__(self):
        self.read = []
        self.write = []


class ActivityStream(object):

    def __init__(self):
        pass


class Operation(enum.Enum):
    """An enumerator representing the operation on a blackboard variable"""

    READ = "READ"
    """Behaviour check has passed, or execution of its action has finished with a successful result."""
    WRITE = "WRITE"
    """Behaviour check has failed, or execution of its action finished with a failed result."""
    VALUE = "RUNNING"
    """Behaviour is in the middle of executing some action, result still pending."""
    INVALID = "INVALID"
    """Behaviour is uninitialised and inactive, i.e. this is the status before first entry, and after a higher priority switch has occurred."""


class ActivityItem(object):

    def __init__(self):
        self.key = None
        self.who = None
        self.operation = None
        self.value_previous = None
        self.value_new = None


class Blackboard(object):
    """
    Key-value store for sharing amongst behaviours.

    Examples:
        You can instantiate the blackboard from anywhere in your program. Even
        disconnected calls will get access to the same data store. For example:

        .. code-block:: python

            def check_foo():
                blackboard = Blackboard()
                assert(blackboard.foo, "bar")

            if __name__ == '__main__':
                blackboard = Blackboard()
                blackboard.foo = "bar"
                check_foo()

        If the key value you are interested in is only known at runtime, then
        you can set/get from the blackboard without the convenient variable style
        access:

        .. code-block:: python

            blackboard = Blackboard()
            result = blackboard.set("foo", "bar")
            foo = blackboard.get("foo")

        The blackboard can also be converted and printed (with highlighting)
        as a string. This is useful for logging and debugging.

        .. code-block:: python

            print(Blackboard())


    .. warning::

       Be careful of key collisions. This implementation leaves this management up to the user.

    Args:
        name: client's not nec. unique, but convenient identifier (stringifies the uuid if None)
        unique_identifier: client's unique identifier for tracking (auto-generates if None)
        read: list of keys this client has permission to read
        write: list of keys this client has permission to write

    .. note::

       Initialisation is not handled in construction, merely registration for tracking
       purposes (and incidentally, access permissions).

    Raises:
        TypeError: if the provided name/unique identifier is not of type str/uuid.UUID
        ValueError: if the unique identifier has already been registered

    Attributes:
        Blackboard.storage: key-value storage
        Blackboard.read: # typing.Dict[str, typing.List[uuid.UUID]]  / key : [unique identifier]
        Blackboard.write: # typing.Dict[str, typing.List[uuid.UUID]]  / key : [unique identifier]
        name: client's, not necessarily unique, identifier for tracking
        unique_identifier: client's unique identifier for tracking
        read: # typing.List[str] / [key]: list of keys this client has permission to read
        write: # typing.List[str] / [key]: list of keys this client has permission to write

    .. seealso::
       * :ref:`Blackboards and Blackboard Behaviours <py-trees-demo-blackboard-program>`
    """
    storage = {}  # Dict[str, Any] / key-value storage
    metadata = {}  # Dict[ str, KeyMetaData ] / key-metadata information
    clients = {}   # Dict[ uuid.UUID, Blackboard] / id-client information

    def __init__(
            self, *,
            name: str=None,
            unique_identifier: uuid.UUID=None,
            read: typing.List[str]=[],
            write: typing.List[str]=[]):
        print("__init__")
        if unique_identifier is None:
            unique_identifier = uuid.uuid4()
        if type(unique_identifier) != uuid.UUID:
            raise TypeError("provided unique identifier is not of type uuid.UUID")
        if name is None or not name:
            name = str(unique_identifier)
        if not isinstance(name, str):
            raise TypeError("provided name is not of type str")
        super().__setattr__("unique_identifier", unique_identifier)
        super().__setattr__("name", name)
        super().__setattr__("read", read)
        for key in read:
            Blackboard.metadata.setdefault(key, KeyMetaData())
            Blackboard.metadata[key].read.append(
                super().__getattribute__("unique_identifier")
            )
        super().__setattr__("write", write)
        for key in write:
            Blackboard.metadata.setdefault(key, KeyMetaData())
            Blackboard.metadata[key].write.append(
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
            ValueError: if the client does not have write access to the variable
        """
        print("__setattr__ [{}][{}]".format(name, value))
        if name not in super().__getattribute__("write"):
            raise ValueError("client does not have write access to '{}'".format(name))
        Blackboard.storage[name] = value

    def __getattr__(self, name):
        """
        Convenience attribute style referencing with checking against
        permissions.

        Raises:
            ValueError: if the client does not have read access to the variable
            AttributeError: if the variable does not yet exist on the blackboard
        """
        print("__getattr__ [{}]".format(name))
        try:
            value = Blackboard.storage[name]
            if name not in super().__getattribute__("read"):
                raise ValueError("client does not have read access to '{}'".format(name))
            return value
        except KeyError as e:
            raise AttributeError("variable '{}' does not yet exist on the blackboard".format(name)) from e

    @staticmethod
    def introspect_blackboard():
        print("-----------------")
        print("Introspect")
        print("-----------------")
        print("  Blackboard.storage:\n{}".format(Blackboard.storage))
        print("  Blackboard.metadata:\n{}".format(Blackboard.metadata))
        # print("  Blackboard.write:\n{}".format(Blackboard.write))

    def set(self, name: str, value: typing.Any, overwrite: bool=True):
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

        Raises:
            ValueError: if the client does not have write access to the variable
            AttributeError: if overwrite was not requested and the variable already exists
        """
        if name not in super().__getattribute__("write"):
            raise ValueError("client does not have write access to '{}'".format(name))
        if not overwrite:
            if name in Blackboard.storage:
                raise AttributeError("variable already exists and overwriting was not requested")
        setattr(self, name, value)
        return True

    def get(self, name):
        """
        Method based accessor to the blackboard variables (as opposed to simply using
        '.<name>').

        Args:
            name: name of the variable to get

        Raises:
            ValueError: if the client does not have read access to the variable
            AttributeError: if the variable does not yet exist on the blackboard
        """
        return getattr(self, name)

    def unset(self, name: str):
        """
        For when you need to unset a blackboard variable, this provides a
        convenient helper method. This is particularly useful for unit
        testing behaviours.

        Args:
            name: name of the variable to unset

        Raises:
            AttributeError: if the variable does not yet exist
        """
        delattr(self, name)

    def __str__(self):
        indent = "  "
        s = console.green + type(self).__name__ + console.reset + "\n"
        s += console.white + indent + "Client Data" + console.reset + "\n"
        keys = ["name", "unique_identifier", "read", "write"]
        s += self.stringify_key_value_pairs(keys, self.__dict__, 2 * indent)
        s += console.white + indent + "Variables" + console.reset + "\n"
        keys = list(dict.fromkeys(self.read + self.write))  # unique list, https://www.peterbe.com/plog/fastest-way-to-uniquify-a-list-in-python-3.6
        s += self.stringify_key_value_pairs(keys, Blackboard.storage, 2 * indent)
        return s

    def stringify_key_value_pairs(self, keys, key_value_dict, indent):
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

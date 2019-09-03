#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import itertools
import typing
import uuid

from . import console


class Blackboard(object):
    """
    `Borg`_ style key-value store for sharing amongst behaviours.

    .. _Borg: http://code.activestate.com/recipes/66531-singleton-we-dont-need-no-stinkin-singleton-the-bo/

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
        read: list of keys this client has permission to read
        write: list of keys this client has permission to write
        unique_identifier: uniquely identify the blackboard client

    .. note:: initialisation

    Raises:
        TypeError: if the provided unique identifier is not uuid.UUID type
        ValueError: if the unique identifier has already been registered

    Attributes:
        Blackboard.storage: key-value storage
        Blackboard.read: # typing.Dict[str, typing.List[uuid.UUID]]  / key : [unique identifier]
        Blackboard.write: # typing.Dict[str, typing.List[uuid.UUID]]  / key : [unique identifier]
        read: # typing.List[str] / [key]: list of keys this client has permission to read
        write: # typing.List[str] / [key]: list of keys this client has permission to write
        unique_identifier: this blackboard client's unique identifier (used to track key-value reading/writing)

    .. seealso::
       * :ref:`Blackboards and Blackboard Behaviours <py-trees-demo-blackboard-program>`
    """
    storage = {}  # key-value storage
    read = {}     # Dict[str, List[uuid.UUID]]  / name : [unique identifier]
    write = {}    # Dict[str, List[uuid.UUID]]  / name : [unique identifier]

    def __init__(
            self,
            read: typing.List[str]=[],
            write: typing.List[str]=[],
            unique_identifier: uuid.UUID=None):
        print("__init__")
        if unique_identifier is None:
            unique_identifier = uuid.uuid4()
        if type(unique_identifier) != uuid.UUID:
            raise TypeError("provided unique identifier is not of type uuid.UUID")
        super().__setattr__("unique_identifier", unique_identifier)
        super().__setattr__("read", read)
        for key in read:
            Blackboard.read.setdefault(key, []).append(
                super().__getattribute__("unique_identifier")
            )
        super().__setattr__("write", write)
        for key in write:
            Blackboard.write.setdefault(key, []).append(
                super().__getattribute__("unique_identifier")
            )

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
        print("  Blackboard.read:\n{}".format(Blackboard.read))
        print("  Blackboard.write:\n{}".format(Blackboard.write))

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
        keys = ["unique_identifier", "read", "write"]
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
                if value is None:
                    value_string = "-"
                    s += console.cyan + indent + '{0: <{1}}'.format(key, max_length + 1) + console.reset + ": " + console.yellow + "{0}\n".format(value_string) + console.reset
                else:
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
    # bb1 = Blackboard()
    # bb1.foo = "bar"
    # unused_result = bb1.foo
    # bb1.introspect()
    bb2 = Blackboard(
        read=['foo', 'bar'],
        write=['foo', 'dude'],
        unique_identifier=uuid.uuid4()
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

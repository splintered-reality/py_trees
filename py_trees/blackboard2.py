#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import itertools
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
        unique_identifier: uniquely identify the blackboard client

    Raises:
        ValueError: if the unique identifier has already been registered

    Attributes:
        Blackboard.storage: key-value storage
        Blackboard.read: # typing.Dict[str, typing.List[uuid.UUID]]  / name : [unique identifier]
        Blackboard.write: # typing.Dict[str, typing.List[uuid.UUID]]  / name : [unique identifier]
        Blackboard.users: # typing.Set(uuid.UUID)
        unique_identifier: this blackboard client's unique identifier (used to track key-value reading/writing)

    .. seealso::
       * :ref:`Blackboards and Blackboard Behaviours <py-trees-demo-blackboard-program>`
    """

    # TODO:
    #   Options:
    #     * [DONE] Generate a UUID if None is passed so there's one flow
    #     * Exclusive locking of read or write access on a variable
    #     * Up front only initialisation of variables (exceptions otherwise)
    #     * Couple blackboards with behaviours so unique_id constribution is not nec.
    #       Ugh...coupling leads to the flying spaghetti monster

    storage = {}  # key-value storage
    read = {}     # Dict[str, List[uuid.UUID]]  / name : [unique identifier]
    write = {}    # Dict[str, List[uuid.UUID]]  / name : [unique identifier]
    users = set()

    def __init__(self, unique_identifier: uuid.UUID=None):
        print("__init__")
        if unique_identifier is None:
            unique_identifier = uuid.uuid4()
        super().__setattr__("unique_identifier", unique_identifier)
        super().__setattr__("read", [])
        super().__setattr__("write", [])
        if unique_identifier in Blackboard.users:
            raise ValueError("a blackboard client with this unique id already exists [{}]".format(unique_identifier))
        Blackboard.users.add(unique_identifier)

    def __setattr__(self, name, value):
        print("__setattr__ [{}][{}]".format(name, value))
        Blackboard.storage[name] = value
        if name not in super().__getattribute__("write"):
            super().__getattribute__("write").append(name)
            Blackboard.write.setdefault(name, []).append(
                super().__getattribute__("unique_identifier")
            )

#    def __getattribute__(self, name):
#        print("__getattribute__ [{}]".format(name))
#        super().__getattribute__(name)
#        print("__getattribute__.done [{}]".format(name))

    def __getattr__(self, name):
        print("__getattr__ [{}]".format(name))
        try:
            value = Blackboard.storage[name]
            if name not in super().__getattribute__("read"):
                super().__getattribute__("read").append(name)
                Blackboard.read.setdefault(name, []).append(
                    super().__getattribute__("unique_identifier")
                )
            return value
        except KeyError:
            raise AttributeError(name)

    def introspect(self):
        print("----------")
        print("Introspect")
        print("----------")
        print("client.unique_identifier : {}".format(super().__getattribute__("unique_identifier")))
        print("client.read : {}".format(super().__getattribute__("read")))
        print("client.write: {}".format(super().__getattribute__("write")))
        print("Blackboard.storage:\n{}".format(Blackboard.storage))
        print("Blackboard.read:\n{}".format(Blackboard.read))
        print("Blackboard.write:\n{}".format(Blackboard.write))
        print("Blackboard.users:\n{}".format(Blackboard.users))

    def set(self, name, value, overwrite=True):
        if not overwrite:
            try:
                getattr(self, name)
                return False
            except AttributeError:
                pass
        setattr(self, name, value)
        return True

    def get(self, name):
        try:
            return getattr(self, name)
        except AttributeError:
            return None

    def unset(self, name):
        """
        For when you need to unset a blackboard variable, this provides a convenient helper method.
        This is particularly useful for unit testing behaviours.

        Args:
            name (:obj:`str`): name of the variable to unset
        """
        try:
            delattr(self, name)
        except AttributeError:
            pass

    def __str__(self):
        s = console.green + type(self).__name__ + "\n" + console.reset
        max_length = 0
        for k in self.__dict__.keys():
            max_length = len(k) if len(k) > max_length else max_length
        keys = sorted(self.__dict__)
        for key in keys:
            value = self.__dict__[key]
            if value is None:
                value_string = "-"
                s += console.cyan + "  " + '{0: <{1}}'.format(key, max_length + 1) + console.reset + ": " + console.yellow + "{0}\n".format(value_string) + console.reset
            else:
                lines = ('{0}'.format(value)).split('\n')
                if len(lines) > 1:
                    s += console.cyan + "  " + '{0: <{1}}'.format(key, max_length + 1) + console.reset + ":\n"
                    for line in lines:
                        s += console.yellow + "    {0}\n".format(line) + console.reset
                else:
                    s += console.cyan + "  " + '{0: <{1}}'.format(key, max_length + 1) + console.reset + ": " + console.yellow + '{0}\n'.format(value) + console.reset
        s += console.reset
        return s


class Foo(object):

    def __init__(self):
        print("foo.__init__")


##############################################################################
# Main
##############################################################################


def main():
    bb1 = Blackboard()
    bb1.foo = "bar"
    unused_result = bb1.foo
    bb1.introspect()
    bb2 = Blackboard(unique_identifier=uuid.uuid4())
    bb2.foo = "more bar"
    bb2.dude = "bob"
    unused_result = bb2.foo
    bb2.introspect()

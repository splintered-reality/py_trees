#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/splintered-reality/py_trees/devel/LICENSE
#

##############################################################################
# Imports
##############################################################################

import nose
import uuid

import py_trees
import py_trees.console as console

from py_trees.blackboard import Blackboard

##############################################################################
# Helpers
##############################################################################


class create_blackboards(object):

    def __enter__(self):
        self.foo = create_blackboard_foo()
        self.bar = create_blackboard_bar()
        return (self.foo, self.bar)

    def __exit__(self, unused_type, unused_value, unused_traceback):
        self.foo.unregister()
        self.bar.unregister()


def create_blackboard_foo():
    """
    Create a blackboard client with a few variables.
    """
    blackboard = Blackboard(
        name="foo",
        unique_identifier=uuid.uuid4(),
        read=['foo', 'bar'],
        write=['foo', 'dude'],
    )
    # blackboard.foo = "more bar"  # leave one uninitialised
    blackboard.dude = "bob"
    return blackboard


def create_blackboard_bar():
    """
    Create another blackboard client with a few variables.
    """
    blackboard = Blackboard(
        name="foo",
        unique_identifier=uuid.uuid4(),
        read=['dude'],
        write=['bar', 'dudette'],
    )
    # blackboard.foo = "more bar"  # leave one uninitialised
    blackboard.bar = "less bar"
    blackboard.dudette = "Anna"
    return blackboard

##############################################################################
# Tests
##############################################################################


def test_client_print_blackboard():
    console.banner("Client Construction & Print")
    with create_blackboards() as (foo, bar):
        print('{0}'.format(foo))
        print('{0}'.format(bar))
    assert(True)


def test_bad_name_exception():
    console.banner("Bad Name Exception")
    with nose.tools.assert_raises_regexp(TypeError, "str"):
        print("Expecting a TypeError with substring 'str'")
        unused_blackboard = py_trees.blackboard.Blackboard(name=5)


def test_bad_uuid_exception():
    console.banner("Bad UUID Exception")
    with nose.tools.assert_raises_regexp(TypeError, "UUID"):
        print("Expecting a TypeError with substring 'UUID'")
        unused_blackboard = py_trees.blackboard.Blackboard(unique_identifier=5)


def test_duplicate_uuid_exception():
    console.banner("Duplicate UUID Exception")
    unique_identifier = uuid.uuid4()
    unused_one = py_trees.blackboard.Blackboard(unique_identifier=unique_identifier)
    with nose.tools.assert_raises_regexp(ValueError, "already been registered"):
        print("Expecting a ValueError with substring 'already been registered'")
        unused_two = py_trees.blackboard.Blackboard(unique_identifier=unique_identifier)


def test_key_filters():
    console.banner("Key Accessors")
    with create_blackboards() as (foo, bar):
        no_of_keys = len(Blackboard.keys())
        print("# Registered keys: {} [{}]".format(no_of_keys, 4))
        assert(no_of_keys == 4)
        no_of_keys = len(Blackboard.keys_filtered_by_regex("dud"))
        print("# Keys by regex 'dud': {} [{}]".format(no_of_keys, 2))
        assert(no_of_keys == 2)
        no_of_keys = len(Blackboard.keys_filtered_by_clients({foo.unique_identifier}))
        print("# Keys by id [foo.id] {} [{}]".format(no_of_keys, 3))
        assert(no_of_keys == 3)
        no_of_keys = len(Blackboard.keys_filtered_by_clients({bar.unique_identifier}))
        print("# Keys by id [bar.id] {} [{}]".format(no_of_keys, 3))
        assert(no_of_keys == 3)
        no_of_keys = len(Blackboard.keys_filtered_by_clients({foo.unique_identifier, bar.unique_identifier}))
        print("# Keys by id [foo.id, bar.id] {} [{}]".format(no_of_keys, 4))
        assert(no_of_keys == 4)
        # show the convenience list -> set helper is ok
        no_of_keys = len(Blackboard.keys_filtered_by_clients([foo.unique_identifier]))
        print("# Can pass in a list instead of a set: True")
        assert(no_of_keys == 3)


def test_activity_stream():
    console.banner("Activity Stream")
    Blackboard.enable_activity_stream(100)
    blackboard = Blackboard(
        name="Client",
        read={"foo", "dude"},
        write={"spaghetti"}
    )
    try:
        unused = blackboard.dude
    except KeyError:  # READ_FAILED
        pass
    try:
        unused = blackboard.dudette
    except AttributeError:  # READ_ACCESS DENIED
        pass
    try:
        blackboard.dudette = "Jane"
    except AttributeError:  # WRITE_ACCESS DENIED
        pass
    blackboard.spaghetti = {"type": "Carbonara", "quantity": 1}  # INITIALISED
    blackboard.spaghetti = {"type": "Gnocchi", "quantity": 2}  # WRITE
    try:
        # NO_OVERWRITE
        blackboard.set("spaghetti", {"type": "Bolognese", "quantity": 3}, overwrite=False)
    except AttributeError:
        pass
    blackboard.unset("spaghetti")  # UNSET
    print(py_trees.display.unicode_blackboard_activity_stream())
    expected_types = [
        py_trees.blackboard.ActivityType.READ_FAILED,
        py_trees.blackboard.ActivityType.READ_DENIED,
        py_trees.blackboard.ActivityType.WRITE_DENIED,
        py_trees.blackboard.ActivityType.INITIALISED,
        py_trees.blackboard.ActivityType.WRITE,
        py_trees.blackboard.ActivityType.NO_OVERWRITE,
        py_trees.blackboard.ActivityType.UNSET,
    ]
    for item, expected in zip(Blackboard.activity_stream.data, expected_types):
        assert(item.activity_type == expected)

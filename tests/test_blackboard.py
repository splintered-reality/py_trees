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

from py_trees.blackboard import Blackboard, BlackboardClient

##############################################################################
# Helpers
##############################################################################


class Motley(object):
    """
    To test nested access on the blackboard
    """
    def __init__(self):
        self.nested = "nested"


class create_blackboards(object):

    def __enter__(self):
        self.foo = create_blackboard_foo()
        self.bar = create_blackboard_bar()
        return (self.foo, self.bar)

    def __exit__(self, unused_type, unused_value, unused_traceback):
        self.foo.unregister(clear=True)
        self.bar.unregister(clear=True)


def create_blackboard_foo():
    """
    Create a blackboard client with a few variables.
    """
    blackboard = BlackboardClient(
        name="foo",
        unique_identifier=uuid.uuid4(),
        read=['foo', 'bar', 'motley'],
        write=['foo', 'dude'],
    )
    # blackboard.foo = "more bar"  # leave one uninitialised
    blackboard.dude = "bob"
    return blackboard


def create_blackboard_bar():
    """
    Create another blackboard client with a few variables.
    """
    blackboard = BlackboardClient(
        name="bar",
        unique_identifier=uuid.uuid4(),
        read=['dude'],
        write=['bar', 'dudette', 'motley'],
    )
    # blackboard.foo = "more bar"  # leave one uninitialised
    blackboard.bar = "less bar"
    blackboard.dudette = "Anna"
    blackboard.motley = Motley()
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
        unused_blackboard = py_trees.blackboard.BlackboardClient(name=5)


def test_bad_uuid_exception():
    console.banner("Bad UUID Exception")
    with nose.tools.assert_raises_regexp(TypeError, "UUID"):
        print("Expecting a TypeError with substring 'UUID'")
        unused_blackboard = py_trees.blackboard.BlackboardClient(unique_identifier=5)


def test_duplicate_uuid_exception():
    console.banner("Duplicate UUID Exception")
    unique_identifier = uuid.uuid4()
    unused_one = py_trees.blackboard.BlackboardClient(unique_identifier=unique_identifier)
    with nose.tools.assert_raises_regexp(ValueError, "already been registered"):
        print("Expecting a ValueError with substring 'already been registered'")
        unused_two = py_trees.blackboard.BlackboardClient(unique_identifier=unique_identifier)


def test_delayed_register_key():
    console.banner("Delayed Register Key")
    with create_blackboards() as (foo, bar):
        with nose.tools.assert_raises_regexp(AttributeError, "does not have read/write access"):
            print("Expecting Attribute Error with substring 'does not have read/write access'")
            print(foo.other)
        with nose.tools.assert_raises_regexp(AttributeError, "does not have write access"):
            print("Expecting Attribute Error with substring 'does not have write access'")
            foo.other = 1
        print("register other for writing")
        foo.register_key(key="other", write=True)
        print(str(foo))
        print("foo.other = 1")
        foo.other = 1
        print("Attempting to read 'other'...")
        with nose.tools.assert_raises_regexp(AttributeError, "does not have read/write access"):
            print("Expecting Attribute Error with substring 'does not have read/write access'")
            unused_result = bar.other
        print("register other for reading")
        bar.register_key(key="other", read=True)
        print(str(bar))
        assert(bar.other == 1)


def test_key_exists():
    console.banner("Nested Read")
    with create_blackboards() as (foo, unused_bar):
        print("foo.exists('dude') [{}][{}]".format(foo.exists("dude"), True))
        assert(foo.exists("dude"), True)
        with nose.tools.assert_raises_regexp(AttributeError, "does not have read/write access"):
            print("Expecting Attribute Error with substring 'does not have read/write access'")
            print("foo.exists('dude_not_here') [{}][{}]".format(foo.exists("dude_not_here"), False))
            assert(foo.exists("dude_not_here"), False)


def test_nested_exists():
    console.banner("Nested Read")
    with create_blackboards() as (foo, unused_bar):
        print("foo.exists('motley.nested') [{}][{}]".format(foo.exists("foo.nested"), True))
        assert(foo.exists("motley.nested"), True)
        print("foo.exists('motley.not_here') [{}][{}]".format(foo.exists("foo.not_here"), False))
        assert(foo.exists("motley.not_here"), False)


def test_nested_read():
    console.banner("Nested Read")
    with create_blackboards() as (foo, unused_bar):
        result = foo.motley.nested
        print("Read foo.motley.nested {} [{}]".format(result, "nested"))
        assert(result == "nested")
        result = foo.get('motley.nested')
        print("Read foo.get('motley.nested') {} [{}]".format(result, "nested"))
        assert(result == "nested")
        with nose.tools.assert_raises_regexp(KeyError, "nested attributes"):
            print("foo.get('motley.huzzah_not_here') ...")
            print("Expecting a KeyError with substring 'nested attributes'")
            foo.get('motley.huzzah_not_here')
        foo.unset('motley')
        with nose.tools.assert_raises_regexp(KeyError, "does not yet exist"):
            print("foo.unset('motley')")
            print("foo.get('motley.not_here') ...")
            print("Expecting a KeyError with substring 'does not yet exist'")
            foo.get('motley.not_here')


def test_nested_write():
    console.banner("Nested Write")
    with create_blackboards() as (foo, bar):
        print("Write bar.motley.nested [{}]".format("overwritten"))
        bar.motley.nested = "overwritten"
        print("  'foo.motley.nested' == {} [{}]".format("overwritten", foo.motley.nested))
        assert(foo.motley.nested == "overwritten")
        print("Write bar.set('motley.nested', {})".format("via_set_overwrite"))
        bar.set('motley.nested', "via_set_overwrite")
        print("  'foo.motley.nested' == {} [{}]".format("via_set_overwrite", foo.motley.nested))
        assert(foo.motley.nested == "via_set_overwrite")
        print("Write bar.set('motley.nested', '{}', overwrite=False)".format("try_to_overwrite"))
        result = bar.set('motley.nested', "try_to_overwrite", overwrite=False)
        print("  'result' == {} [{}]".format(False, result))
        print("  'foo.motley.nested' == {} [{}]".format("via_set_overwrite", foo.motley.nested))
        assert(foo.motley.nested == "via_set_overwrite")
        with nose.tools.assert_raises_regexp(KeyError, "does not yet exist"):
            print("bar.unset('motley')")
            bar.unset('motley')
            print("bar.set('motley.nested', 'on_unset') ...")
            print("Expecting a KeyError with substring 'does not yet exist'")
            bar.set('motley.nested', "on_unset")


def test_key_filters():
    console.banner("Key Accessors")
    with create_blackboards() as (foo, bar):
        no_of_keys = len(Blackboard.keys())
        print("{}".format(Blackboard.keys()))
        print("# Registered keys: {} [{}]".format(no_of_keys, 5))
        assert(no_of_keys == 5)
        no_of_keys = len(Blackboard.keys_filtered_by_regex("dud"))
        print("# Keys by regex 'dud': {} [{}]".format(no_of_keys, 2))
        assert(no_of_keys == 2)
        no_of_keys = len(Blackboard.keys_filtered_by_clients({foo.unique_identifier}))
        print("# Keys by id [foo.id] {} [{}]".format(no_of_keys, 4))
        assert(no_of_keys == 4)
        no_of_keys = len(Blackboard.keys_filtered_by_clients({bar.unique_identifier}))
        print("# Keys by id [bar.id] {} [{}]".format(no_of_keys, 4))
        assert(no_of_keys == 4)
        no_of_keys = len(Blackboard.keys_filtered_by_clients({foo.unique_identifier, bar.unique_identifier}))
        print("# Keys by id [foo.id, bar.id] {} [{}]".format(no_of_keys, 5))
        assert(no_of_keys == 5)
        # show the convenience list -> set helper is ok
        no_of_keys = len(Blackboard.keys_filtered_by_clients([foo.unique_identifier]))
        print("# Can pass in a list instead of a set: True")
        assert(no_of_keys == 4)


def test_activity_stream():
    console.banner("Activity Stream")
    Blackboard.enable_activity_stream(100)
    blackboard = BlackboardClient(
        name="Client",
        read={"foo", "dude"},
        write={"spaghetti", "motley"}
    )
    try:
        unused = blackboard.dude
    except KeyError:  # NO_KEY
        pass
    try:
        unused = blackboard.dudette
    except AttributeError:  # ACCESS_DENIED
        pass
    try:
        blackboard.dudette = "Jane"
    except AttributeError:  # ACCESS_DENIED
        pass
    blackboard.spaghetti = {"type": "Carbonara", "quantity": 1}  # INITIALISED
    blackboard.spaghetti = {"type": "Gnocchi", "quantity": 2}  # WRITE
    blackboard.motley = Motley()
    blackboard.motley.nested = "mutt"
    unused_result = blackboard.motley.nested
    try:
        # NO_OVERWRITE
        blackboard.set("spaghetti", {"type": "Bolognese", "quantity": 3}, overwrite=False)
    except AttributeError:
        pass
    blackboard.unset("spaghetti")  # UNSET
    print(py_trees.display.unicode_blackboard_activity_stream())
    expected_types = [
        py_trees.blackboard.ActivityType.NO_KEY,
        py_trees.blackboard.ActivityType.ACCESS_DENIED,
        py_trees.blackboard.ActivityType.ACCESS_DENIED,
        py_trees.blackboard.ActivityType.INITIALISED,
        py_trees.blackboard.ActivityType.WRITE,
        py_trees.blackboard.ActivityType.INITIALISED,
        py_trees.blackboard.ActivityType.ACCESSED,
        py_trees.blackboard.ActivityType.ACCESSED,
        py_trees.blackboard.ActivityType.NO_OVERWRITE,
        py_trees.blackboard.ActivityType.UNSET,
    ]
    for item, expected in zip(Blackboard.activity_stream.data, expected_types):
        assert(item.activity_type == expected)


def test_lists():
    console.banner("Lists")
    with create_blackboards() as (foo, bar):
        foo.dude = ["Bob", "Bill"]
        name = bar.dude[0]
        print("Read first element of the list: {} [{}]".format(name, "Bob"))
        assert(name, "Bob")


def test_dicts():
    console.banner("Dicts")
    with create_blackboards() as (foo, bar):
        foo.dude = {"Bob": 5, "Bill": 3}
        value = bar.dude["Bob"]
        print("Read Bob's score: {} [{}]".format(value, 5))
        assert(value, 5)


def test_static_get_set():
    console.banner("Blackboard get/set")
    print("Set foo: 5")
    Blackboard.set("foo", 5)
    print("Get foo")
    with nose.tools.assert_raises_regexp(KeyError, "bar"):
        print(" - Expecting a KeyError")
        unused_value = Blackboard.get("bar")
    print("Set motley: Motley()")
    Blackboard.set("motley", Motley())
    print("Set motley.nested: nooo")
    Blackboard.set("motley.nested", "nooo")
    assert(Blackboard.get("motley.nested"), "nooo")
    print("Get motley.foo")
    with nose.tools.assert_raises_regexp(KeyError, "nested attributes"):
        print(" - Expecting a KeyError")
        unused_value = Blackboard.get("motley.foo")
    print("Set motley.other: floosie")
    Blackboard.set("motley.other", "floosie")
    assert(Blackboard.get("motley.other"), "floosie")
    print("Get missing")
    with nose.tools.assert_raises_regexp(KeyError, "missing"):
        print(" - Expecting a KeyError")
        unused_value = Blackboard.get("missing")

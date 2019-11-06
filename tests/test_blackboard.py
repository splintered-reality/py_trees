#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/splintered-reality/py_trees/devel/LICENSE
#

##############################################################################
# Imports
##############################################################################

import nose
import sys

import py_trees
import py_trees.console as console

from py_trees.blackboard import Blackboard

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
        self.namespace = ""
        return (self.foo, self.bar, self.namespace)

    def __exit__(self, unused_type, unused_value, unused_traceback):
        self.foo.unregister(clear=True)
        self.bar.unregister(clear=True)


class create_namespaced_blackboards(object):

    def __enter__(self):
        self.namespace = "woohoo_"
        self.foo = create_blackboard_foo(namespace=self.namespace)
        self.bar = create_blackboard_bar(namespace=self.namespace)
        return (self.foo, self.bar, self.namespace)

    def __exit__(self, unused_type, unused_value, unused_traceback):
        self.foo.unregister(clear=True)
        self.bar.unregister(clear=True)


def create_blackboard_foo(namespace=None):
    """
    Create a blackboard client with a few variables.
    """
    blackboard = py_trees.blackboard.Client(
        name="foo",
        namespace=namespace
    )
    for key in {'foo', 'bar', 'motley'}:
        blackboard.register_key(key=key, access=py_trees.common.Access.READ)
    for key in {'foo', 'dude'}:
        blackboard.register_key(key=key, access=py_trees.common.Access.WRITE)
    # blackboard.foo = "more bar"  # leave one uninitialised
    blackboard.dude = "bob"
    return blackboard


def create_blackboard_bar(namespace=None):
    """
    Create another blackboard client with a few variables.
    """
    blackboard = py_trees.blackboard.Client(
        name="bar",
        namespace=namespace
    )
    for key in {'dude'}:
        blackboard.register_key(key=key, access=py_trees.common.Access.READ)
    for key in {'bar', 'dudette', 'motley'}:
        blackboard.register_key(key=key, access=py_trees.common.Access.WRITE)
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
    with create_blackboards() as (foo, bar, unused_namespace):
        print('{0}'.format(foo))
        print('{0}'.format(bar))
    assert(True)


def test_bad_name_exception():
    console.banner("Bad Name Exception")
    with nose.tools.assert_raises_regexp(TypeError, "str"):
        print("Expecting a TypeError with substring 'str'")
        unused_blackboard = py_trees.blackboard.Client(name=5)


def test_unregister_keys_with_clear():
    console.banner("Unregister Keys with Clear")
    Blackboard.clear()
    print(py_trees.display.unicode_blackboard())
    print(py_trees.display.unicode_blackboard(display_only_key_metadata=True))
    print("--------------------------")
    with create_namespaced_blackboards() as (unused_foo, unused_bar, unused_namespace):
        print(py_trees.display.unicode_blackboard())
        print(py_trees.display.unicode_blackboard(display_only_key_metadata=True))
    print("--------------------------")
    print(py_trees.display.unicode_blackboard())
    print(py_trees.display.unicode_blackboard(display_only_key_metadata=True))
    assert(not Blackboard.storage)
    assert(not Blackboard.metadata)


def test_delayed_register_key():
    console.banner("Delayed Register Key")
    for create in [create_blackboards, create_namespaced_blackboards]:
        with create() as (foo, bar, unused_namespace):
            with nose.tools.assert_raises_regexp(AttributeError, "does not have read/write access"):
                print("Expecting Attribute Error with substring 'does not have read/write access'")
                print(foo.other)
            with nose.tools.assert_raises_regexp(AttributeError, "does not have write access"):
                print("Expecting Attribute Error with substring 'does not have write access'")
                foo.other = 1
            print("register other for writing")
            foo.register_key(key="other", access=py_trees.common.Access.WRITE)
            print(str(foo))
            print("foo.other = 1")
            foo.other = 1
            print("Attempting to read 'other'...")
            with nose.tools.assert_raises_regexp(AttributeError, "does not have read/write access"):
                print("Expecting Attribute Error with substring 'does not have read/write access'")
                unused_result = bar.other
            print("register other for reading")
            bar.register_key(key="other", access=py_trees.common.Access.READ)
            print(str(bar))
            assert(bar.other == 1)


def test_key_exists():
    console.banner("Key Exists")
    for create in [create_blackboards, create_namespaced_blackboards]:
        with create() as (foo, unused_bar, namespace):
            print("foo.exists('dude') [{}][{}]".format(foo.exists("dude"), True))
            assert(foo.exists("dude"))
            if namespace:
                print("Blackboard.exists('{}dude') [{}][{}]".format(
                    namespace,
                    Blackboard.exists(name="{}dude".format(namespace)),
                    True)
                )
                assert(Blackboard.exists(name="{}dude".format(namespace)))
            with nose.tools.assert_raises_regexp(AttributeError, "does not have read/write access"):
                print("Expecting Attribute Error with substring 'does not have read/write access'")
                print("foo.exists('dude_not_here') [{}][{}]".format(foo.exists("dude_not_here"), False))
                assert(not foo.exists("dude_not_here"))


def test_nested_exists():
    console.banner("Nested Read")
    for create in [create_blackboards, create_namespaced_blackboards]:
        with create() as (foo, unused_bar, namespace):
            print("foo.exists('motley.nested') [{}][{}]".format(foo.exists("motley.nested"), True))
            assert(foo.exists("motley.nested"))
            print("foo.exists('motley.not_here') [{}][{}]".format(foo.exists("motley.not_here"), False))
            assert(not foo.exists("motley.not_here"))

            namespaced_name = "{}motley.nested".format(namespace)
            print("Blackboard.exists({}) [{}][{}]".format(
                namespaced_name,
                Blackboard.exists(namespaced_name),
                True)
            )
            assert(Blackboard.exists(namespaced_name))


def test_nested_read():
    console.banner("Nested Read")
    for create in [create_blackboards, create_namespaced_blackboards]:
        with create() as (foo, unused_bar, unused_namespace):
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
    for create in [create_blackboards, create_namespaced_blackboards]:
        with create() as (foo, bar, unused_namespace):
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
    for create in [create_blackboards, create_namespaced_blackboards]:
        with create() as (foo, bar, unused_namespace):
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
    blackboard = py_trees.blackboard.Client(name="Client")
    for key in {'foo', 'dude'}:
        blackboard.register_key(key=key, access=py_trees.common.Access.READ)
    for key in {"spaghetti", "motley"}:
        blackboard.register_key(key=key, access=py_trees.common.Access.WRITE)
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
    blackboard.unregister(clear=True)


def test_lists():
    console.banner("Lists")
    for create in [create_blackboards, create_namespaced_blackboards]:
        with create() as (foo, bar, unused_namespace):
            foo.dude = ["Bob", "Bill"]
            name = bar.dude[0]
            print("Read first element of the list: {} [{}]".format(name, "Bob"))
            assert(name == "Bob")


def test_dicts():
    console.banner("Dicts")
    for create in [create_blackboards, create_namespaced_blackboards]:
        with create() as (foo, bar, unused_namespace):
            foo.dude = {"Bob": 5, "Bill": 3}
            value = bar.dude["Bob"]
            print("Read Bob's score: {} [{}]".format(value, 5))
            assert(value == 5)


def test_static_get_set():
    console.banner("Blackboard get/set")
    print("Set foo: 5")
    Blackboard.set("foo", 5)
    print("Unset foo")
    Blackboard.unset("foo")
    with nose.tools.assert_raises_regexp(KeyError, "foo"):
        print(" - Expecting a KeyError")
        unused_value = Blackboard.get("foo")
    print("Get bar")
    with nose.tools.assert_raises_regexp(KeyError, "bar"):
        print(" - Expecting a KeyError")
        unused_value = Blackboard.get("bar")
    print("Set motley: Motley()")
    Blackboard.set("motley", Motley())
    print("Set motley.nested: nooo")
    Blackboard.set("motley.nested", "nooo")
    assert(Blackboard.get("motley.nested") == "nooo")
    print("Get motley.foo")
    with nose.tools.assert_raises_regexp(KeyError, "nested attributes"):
        print(" - Expecting a KeyError")
        unused_value = Blackboard.get("motley.foo")
    print("Set motley.other: floosie")
    Blackboard.set("motley.other", "floosie")
    assert(Blackboard.get("motley.other") == "floosie")
    print("Get missing")
    with nose.tools.assert_raises_regexp(KeyError, "missing"):
        print(" - Expecting a KeyError")
        unused_value = Blackboard.get("missing")


def test_unregister_key():
    console.banner("Unregister Keys")
    for create in [create_blackboards, create_namespaced_blackboards]:
        with create() as (foo, bar, unused_namespace):
            print("'foo' in foo.write")
            assert ("foo" in foo.write)
            print("Foo unregisters 'foo'")
            foo.unregister_key("foo")
            print("'foo' not in foo.write")
            assert ("foo" not in foo.write)
            print("Bar unregisters 'dudette' with clearing")
            print("'dudette' not in bar.write")
            bar.unregister_key("dudette", clear=True)
            assert("dudette" not in bar.write)
            print("'dudette' not on the blackboard")
            assert("dudette" not in Blackboard.storage)


def test_required_keys():
    console.banner("Required")
    blackboard = py_trees.blackboard.Client(name="Reader")
    blackboard.register_key(
        key="foo",
        access=py_trees.common.Access.READ
    )
    blackboard.register_key(
        key="bar",
        access=py_trees.common.Access.READ,
        required=True
    )
    with nose.tools.assert_raises_regexp(KeyError, "but not yet on the blackboard"):
        print("Key does not exist - expecting a KeyError")
        blackboard.validate_required_keys_exist()

    py_trees.blackboard.Blackboard.set(
        variable_name="bar",
        value="boom"
    )

    try:
        print("Key exists - expecting no KeyError")
        blackboard.validate_required_keys_exist()
    except KeyError:
        assert(False)

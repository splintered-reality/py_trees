#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/splintered-reality/py_trees/devel/LICENSE
#

##############################################################################
# Imports
##############################################################################

import typing

import py_trees
import py_trees.console as console
import py_trees.tests

import pytest

from py_trees.blackboard import Blackboard

##############################################################################
# Helpers
##############################################################################


class Motley(object):
    """
    To test nested access on the blackboard
    """

    def __init__(self) -> None:
        self.nested = "nested"


class create_blackboards(object):
    def __enter__(
        self,
    ) -> typing.Tuple[py_trees.blackboard.Client, py_trees.blackboard.Client, str]:
        self.foo = create_blackboard_foo()
        self.bar = create_blackboard_bar()
        self.namespace = ""
        return (self.foo, self.bar, self.namespace)

    def __exit__(
        self,
        unused_type: typing.Any,
        unused_value: typing.Any,
        unused_traceback: typing.Any,
    ) -> None:
        self.foo.unregister(clear=True)
        self.bar.unregister(clear=True)


class create_namespaced_blackboards(object):
    def __enter__(
        self,
    ) -> typing.Tuple[py_trees.blackboard.Client, py_trees.blackboard.Client, str]:
        self.namespace = "/woohoo"
        self.foo = create_blackboard_foo(namespace=self.namespace)
        self.bar = create_blackboard_bar(namespace=self.namespace)
        return (self.foo, self.bar, self.namespace)

    def __exit__(
        self,
        unused_type: typing.Any,
        unused_value: typing.Any,
        unused_traceback: typing.Any,
    ) -> None:
        self.foo.unregister(clear=True)
        self.bar.unregister(clear=True)


# mypy assistance
BlackboardCreators = typing.List[
    typing.Union[
        typing.Type[create_blackboards], typing.Type[create_namespaced_blackboards]
    ]
]


def blackboard_creators() -> BlackboardCreators:
    return [create_blackboards, create_namespaced_blackboards]


def create_blackboard_foo(
    namespace: typing.Optional[str] = None,
) -> py_trees.blackboard.Client:
    """
    Create a blackboard client with a few variables.
    """
    blackboard = py_trees.blackboard.Client(name="foo", namespace=namespace)
    for key in {"bar", "motley"}:
        blackboard.register_key(key=key, access=py_trees.common.Access.READ)
    for key in {"foo", "dude"}:
        blackboard.register_key(key=key, access=py_trees.common.Access.WRITE)
    # blackboard.foo = "more bar"  # leave one uninitialised
    blackboard.dude = "bob"
    return blackboard


def create_blackboard_bar(
    namespace: typing.Optional[str] = None,
) -> py_trees.blackboard.Client:
    """
    Create another blackboard client with a few variables.
    """
    blackboard = py_trees.blackboard.Client(name="bar", namespace=namespace)
    for key in {"dude"}:
        blackboard.register_key(key=key, access=py_trees.common.Access.READ)
    for key in {"bar", "dudette", "motley"}:
        blackboard.register_key(key=key, access=py_trees.common.Access.WRITE)
    # blackboard.foo = "more bar"  # leave one uninitialised
    blackboard.bar = "less bar"
    blackboard.dudette = "Anna"
    blackboard.motley = Motley()
    return blackboard


##############################################################################
# Tests
##############################################################################


def test_client_print_blackboard() -> None:
    console.banner("Client Construction & Print")
    with create_blackboards() as (foo, bar, unused_namespace):
        print("{0}".format(foo))
        print("{0}".format(bar))
    assert True


def test_bad_name_exception() -> None:
    console.banner("Bad Name Exception")

    with pytest.raises(TypeError) as context:  # if raised, context survives
        # intentional error - silence mypy
        py_trees.blackboard.Client(name=5)  # type: ignore[arg-type]
        py_trees.tests.print_assert_details("TypeError raised", "raised", "not raised")
    py_trees.tests.print_assert_details("TypeError raised", "yes", "yes")
    assert "TypeError" == context.typename
    py_trees.tests.print_assert_details("Substring match", "str", f"{context.value}")
    assert "str" in str(context.value)


def test_unregister_keys_with_clear() -> None:
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
    assert not Blackboard.storage
    assert not Blackboard.metadata


def test_delayed_register_key() -> None:
    console.banner("Delayed Register Key")
    for create in blackboard_creators():
        with create() as (foo, bar, unused_namespace):
            with pytest.raises(
                AttributeError
            ) as context:  # if raised, context survives
                print(
                    "Expecting Attribute Error with substring 'does not have read/write access'"
                )
                print(foo.other)
                py_trees.tests.print_assert_details(
                    "AttributeError raised", "raised", "not raised"
                )
            py_trees.tests.print_assert_details("AttributeError raised", "yes", "yes")
            assert "AttributeError" == context.typename
            py_trees.tests.print_assert_details(
                "Substring match", "does not have read/write access", f"{context.value}"
            )
            assert "does not have read/write access" in str(context.value)

            with pytest.raises(
                AttributeError
            ) as context:  # if raised, context survives
                print(
                    "Expecting Attribute Error with substring 'does not have write access'"
                )
                foo.other = 1
                py_trees.tests.print_assert_details(
                    "AttributeError raised", "raised", "not raised"
                )
            py_trees.tests.print_assert_details("AttributeError raised", "yes", "yes")
            assert "AttributeError" == context.typename
            py_trees.tests.print_assert_details(
                "Substring match", "does not have write access", f"{context.value}"
            )
            assert "does not have write access" in str(context.value)

            print("register other for writing")
            foo.register_key(key="other", access=py_trees.common.Access.WRITE)
            print(str(foo))
            print("foo.other = 1")
            foo.other = 1
            print("Attempting to read 'other'...")

            with pytest.raises(
                AttributeError
            ) as context:  # if raised, context survives
                print(
                    "Expecting Attribute Error with substring 'does not have read/write access'"
                )
                unused_result = bar.other  # noqa: F841 [unused]
                py_trees.tests.print_assert_details(
                    "AttributeError raised", "raised", "not raised"
                )
            py_trees.tests.print_assert_details("AttributeError raised", "yes", "yes")
            assert "AttributeError" == context.typename
            py_trees.tests.print_assert_details(
                "Substring match", "does not have read/write access", f"{context.value}"
            )
            assert "does not have read/write access" in str(context.value)

            print("register other for reading")
            bar.register_key(key="other", access=py_trees.common.Access.READ)
            print(str(bar))
            assert bar.other == 1


def test_is_registered() -> None:
    console.banner("Is Registered")
    blackboard = create_blackboard_foo(namespace="aha")
    print(blackboard)
    for key, access in {
        ("bar", py_trees.common.Access.READ),
        ("foo", py_trees.common.Access.WRITE),
        ("dude", py_trees.common.Access.WRITE),
        ("/aha/bar", py_trees.common.Access.READ),
        ("/aha/foo", py_trees.common.Access.WRITE),
        ("/aha/dude", py_trees.common.Access.WRITE),
    }:
        result = blackboard.is_registered(key)
        print("is_registered({}).......[{}][True]".format(key, result))
        assert result is True
        result = blackboard.is_registered(key, access)
        print("is_registered({}, {}).......[{}][True]".format(key, access, result))
        assert result is True
        access = (
            py_trees.common.Access.READ
            if access == py_trees.common.Access.WRITE
            else py_trees.common.Access.WRITE
        )
        result = blackboard.is_registered(key, access)
        print("is_registered({}, {}).......[{}][False]".format(key, access, result))
        assert result is False
    for key in {"/aha/foobar", "/aha/dudette"}:
        result = blackboard.is_registered(key)
        print("is_registered({}).......[{}][False]".format(key, result))
        assert result is False
    for key in {"/foo", "/dude"}:
        result = blackboard.is_registered(key)
        print("is_registered({}).......[{}][False]".format(key, result))
        assert result is False
    blackboard.unregister(clear=True)


def test_key_exists() -> None:
    console.banner("Key Exists")
    for create in blackboard_creators():
        with create() as (foo, unused_bar, namespace):
            py_trees.tests.print_assert_details(
                "'dude' exists", foo.exists("dude"), True
            )
            assert foo.exists("dude")
            if namespace:
                py_trees.tests.print_assert_details(
                    "'/woohoo/dude' exists", Blackboard.exists("dude"), True
                )
                assert Blackboard.exists(name="{}/dude".format(namespace))

            with pytest.raises(
                AttributeError
            ) as context:  # if raised, context survives
                print("Checking existence of non-existant 'dude_not_here'")
                print(
                    "foo.exists('dude_not_here') [{}][{}]".format(
                        foo.exists("dude_not_here"), False
                    )
                )
                py_trees.tests.print_assert_details(
                    "AttributeError raised", "raised", "not raised"
                )
            py_trees.tests.print_assert_details("AttributeError raised", "yes", "yes")
            assert "AttributeError" == context.typename
            py_trees.tests.print_assert_details(
                "Substring match", "does not have read/write access", f"{context.value}"
            )
            assert "does not have read/write access" in str(context.value)


def test_nested_exists() -> None:
    console.banner("Nested Read")
    for create in blackboard_creators():
        with create() as (foo, unused_bar, namespace):
            print(
                "foo.exists('motley.nested') [{}][{}]".format(
                    foo.exists("motley.nested"), True
                )
            )
            assert foo.exists("motley.nested")
            print(
                "foo.exists('motley.not_here') [{}][{}]".format(
                    foo.exists("motley.not_here"), False
                )
            )
            assert not foo.exists("motley.not_here")

            namespaced_name = "{}/motley.nested".format(namespace)
            print(
                "Blackboard.exists({}) [{}][{}]".format(
                    namespaced_name, Blackboard.exists(namespaced_name), True
                )
            )
            assert Blackboard.exists(namespaced_name)


def test_nested_read() -> None:
    console.banner("Nested Read")
    for create in blackboard_creators():
        with create() as (foo, unused_bar, unused_namespace):
            result = foo.motley.nested
            print("Read foo.motley.nested {} [{}]".format(result, "nested"))
            assert result == "nested"
            result = foo.get("motley.nested")
            print("Read foo.get('motley.nested') {} [{}]".format(result, "nested"))
            assert result == "nested"

            with pytest.raises(KeyError) as context:  # if raised, context survives
                print("foo.get('motley.huzzah_not_here') ...")
                print("Expecting a KeyError with substring 'nested attributes'")
                foo.get("motley.huzzah_not_here")
                py_trees.tests.print_assert_details(
                    "KeyError raised", "raised", "not raised"
                )
            py_trees.tests.print_assert_details("KeyError raised", "yes", "yes")
            assert "KeyError" == context.typename
            py_trees.tests.print_assert_details(
                "  substring match", "nested attributes", f"{context.value}"
            )
            assert "nested attributes" in str(context.value)

            print("foo.unset('motley')")
            foo.unset("motley")

            with pytest.raises(KeyError) as context:  # if raised, context survives
                print("foo.get('motley.not_here') ...")
                print("Expecting a KeyError with substring 'does not yet exist'")
                foo.get("motley.not_here")
                py_trees.tests.print_assert_details(
                    "KeyError raised", "raised", "not raised"
                )
            py_trees.tests.print_assert_details("KeyError raised", "yes", "yes")
            assert "KeyError" == context.typename
            py_trees.tests.print_assert_details(
                "  substring match", "does not yet exist", f"{context.value}"
            )
            assert "does not yet exist" in str(context.value)


def test_nested_write() -> None:
    console.banner("Nested Write")
    for create in blackboard_creators():
        with create() as (foo, bar, unused_namespace):
            print("Write bar.motley.nested [{}]".format("overwritten"))
            bar.motley.nested = "overwritten"
            print(
                "  'foo.motley.nested' == {} [{}]".format(
                    "overwritten", foo.motley.nested
                )
            )
            assert foo.motley.nested == "overwritten"
            print("Write bar.set('motley.nested', {})".format("via_set_overwrite"))
            bar.set("motley.nested", "via_set_overwrite")
            print(
                "  'foo.motley.nested' == {} [{}]".format(
                    "via_set_overwrite", foo.motley.nested
                )
            )
            assert foo.motley.nested == "via_set_overwrite"
            print(
                "Write bar.set('motley.nested', '{}', overwrite=False)".format(
                    "try_to_overwrite"
                )
            )
            result = bar.set("motley.nested", "try_to_overwrite", overwrite=False)
            print("  'result' == {} [{}]".format(False, result))
            print(
                "  'foo.motley.nested' == {} [{}]".format(
                    "via_set_overwrite", foo.motley.nested
                )
            )
            assert foo.motley.nested == "via_set_overwrite"

            with pytest.raises(KeyError) as context:  # if raised, context survives
                print("bar.unset('motley')")
                bar.unset("motley")
                print("bar.set('motley.nested', 'on_unset') ...")
                print("Expecting a KeyError with substring 'does not yet exist'")
                bar.set("motley.nested", "on_unset")
                py_trees.tests.print_assert_details(
                    "KeyError raised", "raised", "not raised"
                )
            py_trees.tests.print_assert_details("KeyError raised", "yes", "yes")
            assert "KeyError" == context.typename
            py_trees.tests.print_assert_details(
                "  substring match", "does not yet exist", f"{context.value}"
            )
            assert "does not yet exist" in str(context.value)


def test_key_filters() -> None:
    console.banner("Key Accessors")
    for create in blackboard_creators():
        with create() as (foo, bar, unused_namespace):
            no_of_keys = len(Blackboard.keys())
            print("{}".format(Blackboard.keys()))
            print("# Registered keys: {} [{}]".format(no_of_keys, 5))
            assert no_of_keys == 5
            no_of_keys = len(Blackboard.keys_filtered_by_regex("dud"))
            print("# Keys by regex 'dud': {} [{}]".format(no_of_keys, 2))
            assert no_of_keys == 2
            no_of_keys = len(
                Blackboard.keys_filtered_by_clients({foo.unique_identifier})
            )
            print("# Keys by id [foo.id] {} [{}]".format(no_of_keys, 4))
            assert no_of_keys == 4
            no_of_keys = len(
                Blackboard.keys_filtered_by_clients({bar.unique_identifier})
            )
            print("# Keys by id [bar.id] {} [{}]".format(no_of_keys, 4))
            assert no_of_keys == 4
            no_of_keys = len(
                Blackboard.keys_filtered_by_clients(
                    {foo.unique_identifier, bar.unique_identifier}
                )
            )
            print("# Keys by id [foo.id, bar.id] {} [{}]".format(no_of_keys, 5))
            assert no_of_keys == 5
            # show the convenience list -> set helper is ok
            no_of_keys = len(
                Blackboard.keys_filtered_by_clients([foo.unique_identifier])
            )
            print("# Can pass in a list instead of a set: True")
            assert no_of_keys == 4


def test_activity_stream() -> None:
    console.banner("Activity Stream")
    Blackboard.enable_activity_stream(100)
    blackboard = py_trees.blackboard.Client(name="Client")
    for key in {"foo", "dude"}:
        blackboard.register_key(key=key, access=py_trees.common.Access.READ)
    for key in {"spaghetti", "motley"}:
        blackboard.register_key(key=key, access=py_trees.common.Access.WRITE)
    try:
        unused = blackboard.dude
    except KeyError:  # NO_KEY
        pass
    try:
        unused = blackboard.dudette  # noqa: F841 [unused]
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
    unused_result = blackboard.motley.nested  # noqa: F841 [unused]
    try:
        # NO_OVERWRITE
        blackboard.set(
            "spaghetti", {"type": "Bolognese", "quantity": 3}, overwrite=False
        )
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
    assert Blackboard.activity_stream is not None
    for item, expected in zip(Blackboard.activity_stream.data, expected_types):
        assert item.activity_type == expected.value
    blackboard.unregister(clear=True)


def test_lists() -> None:
    console.banner("Lists")
    for create in blackboard_creators():
        with create() as (foo, bar, unused_namespace):
            foo.dude = ["Bob", "Bill"]
            name = bar.dude[0]
            print("Read first element of the list: {} [{}]".format(name, "Bob"))
            assert name == "Bob"


def test_dicts() -> None:
    console.banner("Dicts")
    for create in blackboard_creators():
        with create() as (foo, bar, unused_namespace):
            foo.dude = {"Bob": 5, "Bill": 3}
            value = bar.dude["Bob"]
            print("Read Bob's score: {} [{}]".format(value, 5))
            assert value == 5


def test_static_get_set() -> None:
    console.banner("Blackboard get/set")
    print("Set foo: 5")
    Blackboard.set("foo", 5)
    print("Unset foo")
    Blackboard.unset("foo")

    with pytest.raises(KeyError) as context:  # if raised, context survives
        unused_value = Blackboard.get("foo")
        py_trees.tests.print_assert_details("KeyError raised", "raised", "not raised")
    py_trees.tests.print_assert_details("KeyError raised", "yes", "yes")
    assert "KeyError" == context.typename
    py_trees.tests.print_assert_details("  substring match", "foo", f"{context.value}")
    assert "foo" in str(context.value)

    print("Get bar")

    with pytest.raises(KeyError) as context:  # if raised, context survives
        unused_value = Blackboard.get("bar")
        py_trees.tests.print_assert_details("KeyError raised", "raised", "not raised")
    py_trees.tests.print_assert_details("KeyError raised", "yes", "yes")
    assert "KeyError" == context.typename
    py_trees.tests.print_assert_details("  substring match", "bar", f"{context.value}")
    assert "bar" in str(context.value)

    print("Set motley: Motley()")
    Blackboard.set("motley", Motley())
    print("Set motley.nested: nooo")
    Blackboard.set("motley.nested", "nooo")
    assert Blackboard.get("motley.nested") == "nooo"
    print("Get motley.foo")

    with pytest.raises(KeyError) as context:  # if raised, context survives
        unused_value = Blackboard.get("motley.foo")  # noqa: F841 [unused]
        py_trees.tests.print_assert_details("KeyError raised", "raised", "not raised")
    py_trees.tests.print_assert_details("KeyError raised", "yes", "yes")
    assert "KeyError" == context.typename
    py_trees.tests.print_assert_details(
        "  substring match", "motley.foo", f"{context.value}"
    )
    assert "motley.foo" in str(context.value)
    print("Set motley.other: floosie")
    Blackboard.set("motley.other", "floosie")
    assert Blackboard.get("motley.other") == "floosie"
    print("Get missing")

    with pytest.raises(KeyError) as context:  # if raised, context survives
        Blackboard.get("missing")
        py_trees.tests.print_assert_details("KeyError raised", "raised", "not raised")
    py_trees.tests.print_assert_details("KeyError raised", "yes", "yes")
    assert "KeyError" == context.typename
    py_trees.tests.print_assert_details(
        "  substring match", "missing", f"{context.value}"
    )
    assert "missing" in str(context.value)


def test_unregister_key() -> None:
    console.banner("Unregister Keys")
    for create in blackboard_creators():
        with create() as (foo, bar, namespace):
            print("'{}/foo' in foo.write".format(namespace))
            assert "{}/foo".format(namespace) in foo.write
            print("Foo unregisters 'foo'")
            foo.unregister_key("foo")
            print("'{}/foo' not in foo.write".format(namespace))
            assert "{}/foo".format(namespace) not in foo.write
            print("Bar unregisters 'dudette' with clearing")
            print("'{}/dudette' not in bar.write".format(namespace))
            bar.unregister_key("dudette", clear=True)
            assert "{}/dudette".format(namespace) not in bar.write
            print("'{}/dudette' not on the blackboard".format(namespace))
            assert "{}/dudette".format(namespace) not in Blackboard.storage


def test_required_keys() -> None:
    console.banner("Required")
    blackboard = py_trees.blackboard.Client(name="Reader")
    blackboard.register_key(key="foo", access=py_trees.common.Access.READ)
    blackboard.register_key(
        key="bar", access=py_trees.common.Access.READ, required=True
    )

    with pytest.raises(KeyError) as context:  # if raised, context survives
        print("Required key does not exist - expecting a KeyError")
        blackboard.verify_required_keys_exist()
        py_trees.tests.print_assert_details("KeyError raised", "raised", "not raised")
    py_trees.tests.print_assert_details("KeyError raised", "yes", "yes")
    assert "KeyError" == context.typename
    py_trees.tests.print_assert_details(
        "  substring match", "but not yet on the blackboard", f"{context.value}"
    )
    assert "but not yet on the blackboard" in str(context.value)

    py_trees.blackboard.Blackboard.set(variable_name="/bar", value="boom")
    try:
        print("Key exists - expecting no KeyError")
        blackboard.verify_required_keys_exist()
    except KeyError:
        assert False
    blackboard.unregister()


def test_absolute_name() -> None:
    console.banner("Absolute Names")
    # should use Blackboard.separator here, but it's a pita - long and unreadable
    # just update this if the separator ever changes
    test_tuples = [
        # namespace, name, absolute name
        ("/", "foo", "/foo"),
        ("/", "/foo", "/foo"),
        ("/foo", "/bar", "/bar"),  # ignores the namespace
        ("/foo", "bar", "/foo/bar"),
        ("/foo/", "bar", "/foo/bar"),
        ("/foo/", "/foo/bar", "/foo/bar"),
        ("/foo/", "foo/bar", "/foo/foo/bar"),
    ]
    for namespace, key, absolute_name in test_tuples:
        print(
            "[{}][{}]..........[{}][{}]".format(
                namespace, key, absolute_name, Blackboard.absolute_name(namespace, key)
            )
        )
        assert absolute_name == Blackboard.absolute_name(namespace, key)


def test_relative_name() -> None:
    console.banner("Relative Names")
    # should use Blackboard.separator here, but it's a pita - long and unreadable
    # just update this if the separator ever changes
    test_tuples = [
        # namespace, name, relative name
        ("/", "foo", "foo"),
        ("/", "/foo", "foo"),
        ("/foo", "bar", "bar"),
        ("/foo/", "bar", "bar"),
        ("/foo", "/foo/bar", "bar"),
        ("/foo/", "/foo/bar", "bar"),
    ]
    for namespace, key, absolute_name in test_tuples:
        print(
            "[{}][{}]..........[{}][{}]".format(
                namespace, key, absolute_name, Blackboard.absolute_name(namespace, key)
            )
        )
        assert absolute_name == Blackboard.relative_name(namespace, key)

    namespace = "/bar"
    key = "/foo/bar"

    with pytest.raises(KeyError) as context:  # if raised, context survives
        Blackboard.relative_name(namespace, key)
        py_trees.tests.print_assert_details("KeyError raised", "raised", "not raised")
    py_trees.tests.print_assert_details("KeyError raised", "yes", "yes")
    assert "KeyError" == context.typename
    py_trees.tests.print_assert_details(
        "  substring match", "/foo/bar", f"{context.value}"
    )
    assert "/foo/bar" in str(context.value)


def test_client_absolute_name() -> None:
    console.banner("Absolute Names from Client API")
    # should use Blackboard.separator here, but it's a pita - long and unreadable
    # just update this if the separator ever changes
    test_tuples = [
        # namespace, name, absolute name
        (None, "foo", "/foo"),
        ("foo", "bar", "/foo/bar"),
        ("foo", "/foo/bar", "/foo/bar"),  # ignores the namespace
        ("/foo/", "foo/bar", "/foo/foo/bar"),
    ]
    for namespace, key, absolute_name in test_tuples:
        blackboard = py_trees.blackboard.Client(name="Blackboard", namespace=namespace)
        blackboard.register_key(key=key, access=py_trees.common.Access.READ)
        print(
            "[{}][{}]..........[{}][{}]".format(
                namespace, key, absolute_name, blackboard.absolute_name(key)
            )
        )
        assert absolute_name == blackboard.absolute_name(key)


def test_namespaced_dot_access() -> None:
    console.banner("Namespaced Dot Access")
    blackboard = py_trees.blackboard.Client(name="Blackboard")
    blackboard.register_key(key="dude", access=py_trees.common.Access.WRITE)
    blackboard.register_key(key="/dudette", access=py_trees.common.Access.WRITE)
    blackboard.register_key(key="/foo/bar/wow", access=py_trees.common.Access.WRITE)
    print("Trying....'blackboard.dude = True'")
    blackboard.dude = True
    assert blackboard.dude
    print("Trying....'blackboard.dude = True'")
    blackboard.dudette = True
    assert blackboard.dudette
    print("Trying....'blackboard.foo.bar.wow = True'")
    blackboard.foo.bar.wow = True
    assert Blackboard.exists("/foo/bar/wow")
    assert blackboard.foo.bar.wow
    blackboard.unregister()


def test_remappings() -> None:
    console.banner("Remappings")
    blackboard = py_trees.blackboard.Client(name="Blackboard")
    blackboard.register_key(key="dude", access=py_trees.common.Access.WRITE)
    blackboard.register_key(key="/dudette", access=py_trees.common.Access.WRITE)
    blackboard.register_key(
        key="/foo/bar/wow",
        access=py_trees.common.Access.WRITE,
        remap_to="/parameters/anticipation",
    )
    print("setters & getters...")
    blackboard.set("/foo/bar/wow", "set")
    assert Blackboard.storage["/parameters/anticipation"] == "set"
    blackboard.foo.bar.wow = "pythonic"
    print("  blackboard.foo.bar.wow == 'pythonic'")
    assert blackboard.foo.bar.wow == "pythonic"
    print("  blackboard.get('/foo/bar/wow') == 'pythonic'")
    assert blackboard.get("/foo/bar/wow") == "pythonic"
    print("  Blackboard.storage['/parameters/anticipation'] == 'pythonic'")
    assert Blackboard.storage["/parameters/anticipation"] == "pythonic"

    print("exists....")
    print("  blackboard.exists('/foo/bar/wow')")
    assert blackboard.exists("/foo/bar/wow")
    print("  Blackboard.exists('/parameters/anticipation')")
    assert Blackboard.exists("/parameters/anticipation")

    print("unset...")
    blackboard.unset("/foo/bar/wow")
    print("  not blackboard.exists('/foo/bar/wow')")
    assert not blackboard.exists("/foo/bar/wow")
    print("  not Blackboard.exists('/parameters/anticipation')")
    assert not Blackboard.exists("/parameters/anticipation")

    print("unregister_key...")
    blackboard.foo.bar.wow = "pythonic"
    blackboard.unregister_key("/foo/bar/wow", clear=True)
    print("  not Blackboard.exists('/parameters/anticipation')")
    assert not Blackboard.exists("/parameters/anticipation")

    print(py_trees.display.unicode_blackboard())
    print(blackboard)
    blackboard.unregister(clear=True)


def test_exclusive_write() -> None:
    console.banner("Exclusive Write")
    blackboard = py_trees.blackboard.Client(name="Blackboard")
    blackboard.register_key(key="dude", access=py_trees.common.Access.EXCLUSIVE_WRITE)
    print("setters & getters...")
    blackboard.dude = "Bob"
    print("  blackboard.dude == 'Bob'")
    assert blackboard.dude == "Bob"
    blackboard.unregister(clear=True)

    print("exclusive fail...")
    blackboard = py_trees.blackboard.Client(name="Blackboard")
    blackboard.register_key(key="dude", access=py_trees.common.Access.WRITE)
    blackboard.register_key(
        key="dudette", access=py_trees.common.Access.EXCLUSIVE_WRITE
    )
    blackboard_exclusive = py_trees.blackboard.Client(name="BlackboardX")

    with pytest.raises(AttributeError) as context:  # if raised, context survives
        print(
            "Exclusive write requested, but already has a writer - expecting an AttributeError"
        )
        blackboard_exclusive.register_key(
            key="dude", access=py_trees.common.Access.EXCLUSIVE_WRITE
        )
        py_trees.tests.print_assert_details(
            "AttributeError raised", "raised", "not raised"
        )
    py_trees.tests.print_assert_details("AttributeError raised", "yes", "yes")
    assert "AttributeError" == context.typename
    py_trees.tests.print_assert_details(
        "  substring match", "requested exclusive write", f"{context.value}"
    )
    assert "requested exclusive write" in str(context.value)

    with pytest.raises(AttributeError) as context:  # if raised, context survives
        print(
            "Exclusive write requested, but already has a writer - expecting an AttributeError"
        )
        blackboard_exclusive.register_key(
            key="dudette", access=py_trees.common.Access.EXCLUSIVE_WRITE
        )
        py_trees.tests.print_assert_details(
            "AttributeError raised", "raised", "not raised"
        )
    py_trees.tests.print_assert_details("AttributeError raised", "yes", "yes")
    assert "AttributeError" == context.typename
    py_trees.tests.print_assert_details(
        "  substring match", "requested exclusive write", f"{context.value}"
    )
    assert "requested exclusive write" in str(context.value)

    with pytest.raises(AttributeError) as context:  # if raised, context survives
        print(
            "Write requested, but already has an exclusive writer - expecting an AttributeError"
        )
        blackboard_exclusive.register_key(
            key="dudette", access=py_trees.common.Access.WRITE
        )
        py_trees.tests.print_assert_details(
            "AttributeError raised", "raised", "not raised"
        )
    py_trees.tests.print_assert_details("AttributeError raised", "yes", "yes")
    assert "AttributeError" == context.typename
    py_trees.tests.print_assert_details(
        "  substring match", "requested write on", f"{context.value}"
    )
    assert "requested write on" in str(context.value)

    blackboard.unregister(clear=True)
    blackboard_exclusive.unregister(clear=True)


def test_blackboard_static_names() -> None:
    console.banner("Test Absolute Names with Static Methods")
    print("Set 'foo'")
    Blackboard.set("foo", "foo")
    print("Get 'foo' [{}]".format(Blackboard.get("foo")))
    assert Blackboard.get("foo") == "foo"
    assert Blackboard.get("/foo") == "foo"
    print("Set '/bar'")
    Blackboard.set("/bar", "bar")
    print("Get 'bar' [{}]".format(Blackboard.get("bar")))
    assert Blackboard.get("bar") == "bar"
    assert Blackboard.get("/bar") == "bar"

#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/splintered-reality/py_trees/devel/LICENSE
#

##############################################################################
# Imports
##############################################################################

import unittest
from enum import Enum
from typing import Any

from py_trees.ports_utils import apply_type_hints, convert_str_to_type


class StdoutLogger:
    """Simple stdout logger matching the PortsLogger protocol."""

    def debug(self, msg: str) -> None:
        print(f"[DEBUG] {msg}")

    def info(self, msg: str) -> None:
        print(f"[INFO] {msg}")

    def warning(self, msg: str) -> None:
        print(f"[WARNING] {msg}")

    def error(self, msg: str) -> None:
        print(f"[ERROR] {msg}")


class ColorInt(Enum):
    RED = 1
    GREEN = 2
    BLUE = 3


class ColorStr(Enum):
    RED = "red"
    GREEN = "green"
    BLUE = "blue"


class TestConvertStrToType(unittest.TestCase):
    def test_primitives_and_enums(self) -> None:
        self.assertEqual(convert_str_to_type("true", bool), (True, True))
        self.assertEqual(convert_str_to_type("False", bool), (True, False))
        self.assertEqual(convert_str_to_type(" 7 ", int), (True, 7))
        success, value = convert_str_to_type("2.5", float)
        self.assertTrue(success)
        self.assertAlmostEqual(value, 2.5)
        self.assertEqual(convert_str_to_type("hello", str), (True, "hello"))

        # enum by NAME (case-insensitive)
        self.assertEqual(convert_str_to_type("green", ColorInt), (True, ColorInt.GREEN))
        self.assertEqual(convert_str_to_type("BLUE", ColorStr), (True, ColorStr.BLUE))

        # enum by VALUE (int-backed / str-backed)
        self.assertEqual(convert_str_to_type("2", ColorInt), (True, ColorInt.GREEN))
        self.assertEqual(convert_str_to_type("blue", ColorStr), (True, ColorStr.BLUE))

    def test_type_target(self) -> None:
        # target_type `type` resolves the string to an actual type object.
        self.assertEqual(convert_str_to_type("float", type), (True, float))
        self.assertEqual(convert_str_to_type("int", type), (True, int))
        self.assertEqual(convert_str_to_type("str", type), (True, str))
        self.assertEqual(convert_str_to_type("bool", type), (True, bool))

    def test_optional_union_lists_tuples(self) -> None:
        self.assertEqual(convert_str_to_type("", int | None), (True, None))
        self.assertEqual(convert_str_to_type("None", int | None), (True, None))
        self.assertEqual(convert_str_to_type("42", int | str), (True, 42))
        self.assertEqual(convert_str_to_type("forty-two", int | str), (True, "forty-two"))

        self.assertEqual(convert_str_to_type("1, 2,3", list[int]), (True, [1, 2, 3]))
        self.assertEqual(
            convert_str_to_type("RED, green", list[ColorInt]),
            (True, [ColorInt.RED, ColorInt.GREEN]),
        )

        self.assertEqual(
            convert_str_to_type("9, hello, true", tuple[int, str, bool]),
            (True, (9, "hello", True)),
        )

    def test_union_member_order_does_not_matter(self) -> None:
        # An unsupported member reports failure without raising, so it must not stop the
        # remaining members from being tried.
        self.assertEqual(convert_str_to_type("5", dict | int), (True, 5))
        self.assertEqual(convert_str_to_type("5", int | dict), (True, 5))

    def test_union_with_str_fallback_reports_success(self) -> None:
        # A value that fails the enum branch but matches the trailing `str` branch of a
        # Union is a legitimate successful conversion, not a failure.
        success, value = convert_str_to_type("not-an-enum", ColorInt | str)
        self.assertTrue(success)
        self.assertEqual(value, "not-an-enum")
        self.assertIsInstance(value, str)


class DummyCtorTarget:
    def __init__(
        self,
        a_int: int,
        b_float: float,
        c_str: str,
        d_bool: bool,
        e_color: ColorInt,
        f_opt_int: int | None,
        g_union_num: int | float,
        h_union_enum: ColorInt | str,
        i_default_str: str = "default",
        **kwargs: int | str,
    ):
        # not used; we only need type hints from the signature
        pass


class TestApplyTypeHints(unittest.TestCase):
    def setUp(self) -> None:
        self.logger = StdoutLogger()
        return super().setUp()

    def test_fallback_on_failure(self) -> None:
        def ctor_with_hints(a: int, b: float, c: bool):  # pragma: no cover - only signature used
            ...

        raw = {"a": "oops", "b": "2.0", "c": "true"}
        success, converted = apply_type_hints(ctor_with_hints, raw, self.logger)
        self.assertFalse(success)
        self.assertEqual(converted["a"], "oops")
        self.assertEqual(converted["b"], 2.0)
        self.assertEqual(converted["c"], True)

    def test_primitives_and_bool(self) -> None:
        raw: dict[str, Any] = {
            "a_int": "42",
            "b_float": "3.125",
            "c_str": "hello",
            "d_bool": "true",
            "e_color": "RED",
            "f_opt_int": "none",
            "g_union_num": "10",
            "h_union_enum": "GREEN",
        }
        success, converted = apply_type_hints(DummyCtorTarget, raw, self.logger)
        self.assertTrue(success)
        self.assertEqual(converted["a_int"], 42)
        self.assertEqual(converted["d_bool"], True)
        self.assertIsNone(converted["f_opt_int"])
        self.assertEqual(converted["h_union_enum"], ColorInt.GREEN)

    def test_union_falling_through_to_str_is_success(self) -> None:
        # Previously flagged as failure since the converted value looked "unchanged".
        raw = {
            "a_int": "1",
            "b_float": "0",
            "c_str": "x",
            "d_bool": "true",
            "e_color": "RED",
            "f_opt_int": "None",
            "g_union_num": "1",
            "h_union_enum": "not-an-enum",
        }
        success, converted = apply_type_hints(DummyCtorTarget, raw, self.logger)
        self.assertTrue(success)
        self.assertEqual(converted["h_union_enum"], "not-an-enum")
        self.assertIsInstance(converted["h_union_enum"], str)

    def test_kwargs_type_fallback(self) -> None:
        # Keys not in the signature (e.g. dynamic kwargs) should fall back to the
        # constructor's **kwargs annotation instead of being reported as unconvertible.
        raw = {
            "a_int": "1",
            "b_float": "0",
            "c_str": "x",
            "d_bool": "true",
            "e_color": "RED",
            "f_opt_int": "None",
            "g_union_num": "1",
            "h_union_enum": "RED",
            "unexpected_kwarg": "should remain string",
        }
        success, converted = apply_type_hints(DummyCtorTarget, raw, self.logger)
        self.assertTrue(success)
        self.assertEqual(converted["unexpected_kwarg"], "should remain string")
        self.assertIsInstance(converted["unexpected_kwarg"], str)

        raw["unexpected_kwarg"] = "123"
        success, converted = apply_type_hints(DummyCtorTarget, raw, self.logger)
        self.assertTrue(success)
        self.assertEqual(converted["unexpected_kwarg"], 123)
        self.assertIsInstance(converted["unexpected_kwarg"], int)

    def test_any_kwargs_catch_all_is_a_pass_through(self) -> None:
        # `**kwargs: Any` is the convention throughout py_trees, so an unknown attribute must
        # stay a string and still count as a success. Note that on Python 3.10 `typing.Any` is
        # not a class, so this also covers conversion of a non-class annotation.
        class AnyKwargsTarget:
            def __init__(self, name: str, **kwargs: Any):  # pragma: no cover - only signature used
                pass

        raw = {"name": "node", "unhinted": "42"}
        success, converted = apply_type_hints(AnyKwargsTarget, raw, self.logger)
        self.assertTrue(success)
        self.assertEqual(converted["unhinted"], "42")
        self.assertIsInstance(converted["unhinted"], str)

    def test_no_type_hint_available(self) -> None:
        raw = {"totally_unknown": "value"}

        class NoKwargsTarget:
            def __init__(self, a_int: int):  # pragma: no cover - only signature used
                pass

        success, converted = apply_type_hints(NoKwargsTarget, raw, self.logger)
        self.assertFalse(success)
        self.assertEqual(converted["totally_unknown"], "value")


if __name__ == "__main__":
    unittest.main()

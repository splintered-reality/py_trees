#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/splintered-reality/py_trees/devel/LICENSE
#

##############################################################################
# Imports
##############################################################################

import re
import unittest

from py_trees.ports_utils import generate_node_name, get_base_name, strip_trailing_uuid4, uuid4_regex

##############################################################################
# Tests
##############################################################################

UUID = "3f2b8c1e-4d5a-4b6c-8d7e-9f0a1b2c3d4e"


class TestUuid4Regex(unittest.TestCase):
    def test_regex_without_anchor_matches_anywhere(self) -> None:
        pattern = uuid4_regex()
        self.assertNotEqual(pattern, "")
        self.assertFalse(pattern.endswith("$"))
        self.assertIsNotNone(re.search(pattern, f"Tree_{UUID}.Node"))

    def test_regex_with_anchor_matches_only_at_end(self) -> None:
        pattern = uuid4_regex(at_end=True)
        self.assertTrue(pattern.endswith("$"))
        self.assertIsNone(re.search(pattern, f"Tree_{UUID}.Node"))
        self.assertIsNotNone(re.search(pattern, f"Tree.Node_{UUID}"))


class TestStripTrailingUuid4(unittest.TestCase):
    def test_trailing_uuid_is_stripped_with_its_separator(self) -> None:
        self.assertEqual(strip_trailing_uuid4(f"Tree.Node_{UUID}"), "Tree.Node")
        self.assertEqual(strip_trailing_uuid4(f"Tree.Node{UUID.upper()}"), "Tree.Node")
        self.assertEqual(strip_trailing_uuid4(UUID), "")

    def test_name_without_trailing_uuid_is_unchanged(self) -> None:
        self.assertEqual(strip_trailing_uuid4("Tree.Node"), "Tree.Node")
        self.assertEqual(strip_trailing_uuid4(f"Tree_{UUID}.Node"), f"Tree_{UUID}.Node")
        self.assertEqual(strip_trailing_uuid4(""), "")

    def test_long_name_is_stripped_only_at_its_end(self) -> None:
        path = ".".join(f"SubTree_{UUID}" for _ in range(40))
        self.assertEqual(strip_trailing_uuid4(f"{path}.Node_{UUID}"), f"{path}.Node")

    def test_uuid_added_by_generate_node_name_is_stripped(self) -> None:
        path = ".".join(["SubTree"] * 50)
        self.assertEqual(strip_trailing_uuid4(generate_node_name(None, "Sequence", prefix=path)), f"{path}.Sequence")


class TestGetBaseName(unittest.TestCase):
    def test_base_name_with_and_without_uuid_stripping(self) -> None:
        self.assertEqual(get_base_name(f"Tree.Node_{UUID}", strip_uuid=True), "Node")
        self.assertEqual(get_base_name(f"Tree.Node_{UUID}"), f"Node_{UUID}")


if __name__ == "__main__":
    unittest.main()

#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/splintered-reality/py_trees/devel/LICENSE
#

##############################################################################
# Imports
##############################################################################

import unittest

import py_trees
from py_trees.ports import (
    BehaviourWithPorts,
    NoDataAvailable,
    get_ports_registry,
    register_ports_class,
)

from .test_ports_helpers import Consumer, ConsumerProducer, Producer


# TODO: Add more tests for PortsMixin methods as needed. There are also some tests that can be ported over from
# test_behavior_with_ports.py to here.
class TestPortsMixin(unittest.TestCase):
    def setUp(self) -> None:
        self.mixin = Producer("test")  # Use Producer class so we don't have to worry about PortsMixin abstract methods

    def test_basic_types(self) -> None:
        self.assertTrue(self.mixin._is_instance_of_type(5, int))
        self.assertTrue(self.mixin._is_instance_of_type(3.14, float))
        self.assertTrue(self.mixin._is_instance_of_type("hello", str))
        self.assertFalse(self.mixin._is_instance_of_type("5", int))
        self.assertFalse(self.mixin._is_instance_of_type(5, str))

    def test_list_of_int(self) -> None:
        self.assertTrue(self.mixin._is_instance_of_type([1, 2, 3], list[int]))
        self.assertFalse(self.mixin._is_instance_of_type([1, "2", 3], list[int]))
        self.assertTrue(self.mixin._is_instance_of_type([], list[int]))  # empty list is valid

    def test_union_type(self) -> None:
        T = int | str
        self.assertTrue(self.mixin._is_instance_of_type(5, T))
        self.assertTrue(self.mixin._is_instance_of_type("hello", T))
        self.assertFalse(self.mixin._is_instance_of_type(3.14, T))

    def test_list_of_union(self) -> None:
        T = list[int | str]
        self.assertTrue(self.mixin._is_instance_of_type([1, "a", 2], T))
        self.assertFalse(self.mixin._is_instance_of_type([1, 2.0], T))

    def test_or_operator(self) -> None:
        T = int | str
        self.assertTrue(self.mixin._is_instance_of_type(5, T))
        self.assertTrue(self.mixin._is_instance_of_type("hello", T))
        self.assertFalse(self.mixin._is_instance_of_type(3.14, T))

        T_list = list[int | str]
        self.assertTrue(self.mixin._is_instance_of_type([1, "a", 2], T_list))
        self.assertFalse(self.mixin._is_instance_of_type([1, 2.0], T_list))

    def test_list_or_element(self) -> None:
        T = int | list[int]
        self.assertTrue(self.mixin._is_instance_of_type(5, T))
        self.assertTrue(self.mixin._is_instance_of_type([1, 2, 3], T))
        self.assertFalse(self.mixin._is_instance_of_type("hello", T))
        self.assertFalse(self.mixin._is_instance_of_type([1, "2"], T))

    def test_not_implemented_for_dict(self) -> None:
        with self.assertRaises(NotImplementedError):
            self.mixin._is_instance_of_type({"a": 1}, dict[str, int])


class TestBehaviourWithPorts(unittest.TestCase):
    def setUp(self) -> None:
        """Reset the blackboard before each test."""
        py_trees.blackboard.Blackboard.clear()

    def test_faulty_output_type(self) -> None:
        """Test that setting an output with an incorrect type raises a TypeError."""
        cap = Producer("FaultyOutput")
        cap.setup_ports()
        with self.assertRaises(TypeError):
            cap._set_output("output", 123)

    def test_faulty_input_type(self) -> None:
        """Test that getting an input with an incorrect type raises a TypeError."""
        cap = Consumer("FaultyInput")
        cap.setup_ports(port_remappings={"input": "/sometest/input"})
        blackboard_client = py_trees.blackboard.Client(name="SomeoneElse")
        blackboard_client.register_key(key="/sometest/input", access=py_trees.common.Access.WRITE, required=True)
        blackboard_client.set("/sometest/input", 123)
        with self.assertRaises(TypeError):
            cap.get_input("input")

    def test_simple_remapping(self) -> None:
        """Test that a simple port remapping between producer and consumer works as expected."""
        # Producer out -> /shared, Consumer in -> /shared
        prod = Producer("prod")
        cons = Consumer("cons")
        prod.setup_ports(port_remappings={"output": "/shared"})
        cons.setup_ports(port_remappings={"input": "/shared"})
        prod._set_output("output", "HelloWorld")
        self.assertEqual(cons.get_input("input"), "HelloWorld")

    def test_default_ports_are_unique_per_node(self) -> None:
        """Ports without explicit remapping should not collide between sibling nodes."""
        prod_a = Producer("duplicate_name")
        prod_b = Producer("duplicate_name")

        prod_a.setup_ports(subtree_namespace="/shared_ns")
        prod_b.setup_ports(subtree_namespace="/shared_ns")

        prod_a._set_output("output", "value_a")
        prod_b._set_output("output", "value_b")

        self.assertEqual(prod_a.get_last_output("output"), "value_a")
        self.assertEqual(prod_b.get_last_output("output"), "value_b")
        self.assertNotEqual(prod_a._get_blackboard_key("output"), prod_b._get_blackboard_key("output"))

    def test_multilevel_remapping(self) -> None:
        """Test that multi-level port remapping through nested subtrees propagates values correctly."""
        # Producer out -> /shared, Consumer in -> /shared
        prod = Producer("prod")
        prod.setup_ports(port_remappings={"output": "/root"}, subtree_namespace="/")
        # Set up a subtree with 2 ConsumerProducers. Note: "transfer" is a relative
        # remap — it is scoped under the node's subtree namespace (here /subtree1),
        # so it is safe to reuse the same name in a sibling subtree without
        # collision. For data that needs to cross subtree boundaries, use an
        # absolute key (starts with "/"), as done with "/subtree1_output" below.
        sbtr1_consprod1 = ConsumerProducer("sbtr1_consprod1")
        sbtr1_consprod2 = ConsumerProducer("sbtr1_consprod2")
        sbtr1_consprod1.setup_ports(
            port_remappings={"input": "/root", "output": "transfer"},
            subtree_namespace="/subtree1",
        )
        sbtr1_consprod2.setup_ports(
            port_remappings={"input": "transfer", "output": "/subtree1_output"},
            subtree_namespace="/subtree1",
        )

        # Set up another nested subtree (grandchild) with 2 ConsumerProducers.
        # It reuses the relative key "transfer" internally — scoped under
        # /subtree1/subtree2, so it does NOT collide with /subtree1/transfer.
        sbtr2_consprod1 = ConsumerProducer("sbtr2_consprod1")
        sbtr2_consprod2 = ConsumerProducer("sbtr2_consprod2")
        sbtr2_consprod1.setup_ports(
            port_remappings={"input": "/subtree1_output", "output": "transfer"},
            subtree_namespace="/subtree1/subtree2",
        )
        sbtr2_consprod2.setup_ports(
            port_remappings={"input": "transfer", "output": "/result"},
            subtree_namespace="/subtree1/subtree2",
        )

        # Set up a consumer in the root namespace
        cons = Consumer("cons")
        cons.setup_ports(port_remappings={"input": "/result"}, subtree_namespace="/")

        # Test that the output is as expected. Tick the tree to make sure that the values are propagated through
        # the tree.
        prod.tick_once()  # ticking should output the value to the blackboard
        sbtr1_consprod1.tick_once()
        sbtr1_consprod2.tick_once()
        sbtr2_consprod1.tick_once()
        sbtr2_consprod2.tick_once()
        cons.tick_once()
        expected_output = (
            "Producer[/:prod]"
            + "[/subtree1:sbtr1_consprod1]"
            + "[/subtree1:sbtr1_consprod2]"
            + "[/subtree1/subtree2:sbtr2_consprod1]"
            + "[/subtree1/subtree2:sbtr2_consprod2]"
        )
        self.assertEqual(cons.get_input("input"), expected_output)

    def test_type_checking(self) -> None:
        """Test that type checking is enforced when setting output values."""
        prod = Producer("prod")
        cons = Consumer("cons")
        prod.setup_ports(port_remappings={"output": "/shared"})
        cons.setup_ports(port_remappings={"input": "/shared"})
        with self.assertRaises(TypeError):
            prod._set_output("output", 123)

    def test_subtree_namespace_remapping(self) -> None:
        """Test that remapping a port to the subtree namespace behaves as expected and is accessible via both keys."""
        # If the remapping key is the subtree namespace + port, there should be no remap_to
        subtree_ns = "/mysubtree"
        port = "output"
        key = "/mysubtree/output"
        prod = Producer("prod")
        prod.setup_ports(port_remappings={port: key}, subtree_namespace=subtree_ns)
        # The blackboard should have a key registered for 'output' with no remap_to
        # This is not directly exposed, so we check that writing to 'output' is accessible as 'output' in the blackboard
        prod._set_output(port, "HelloWorld")
        self.assertEqual(prod.blackboard_client.get(port), "HelloWorld")
        # We can also get the value via the full key
        self.assertEqual(prod.blackboard_client.get(key), "HelloWorld")
        # The blackboard does NOT have any remappings, because the namespace would have been stripped.

        # In the current pytrees implementation, the remapping is still stored - we can only see in the `print`
        # statement that it will be shown as "not remapped".
        # Unfortunately, there is no better way to test this right now.
        # print(prod.blackboard_client)
        self.assertEqual(prod.blackboard_client.remappings, {key: key})

    def test_get_input_with_default(self) -> None:
        """Test that get_input returns the default value when no value is set on the blackboard."""
        cons = Consumer("cons")
        cons.setup_ports(port_remappings={"input": "/shared"})
        # No value is set on the blackboard for the input port
        default_value = "DefaultValue"
        self.assertEqual(cons.get_input("input", default=default_value), default_value)

        # Now set a value and ensure it overrides the default. We also need to re-register the key because
        # input ports are only registered with read access.
        cons.blackboard_client.register_key(key="/shared", access=py_trees.common.Access.WRITE, required=True)
        cons.blackboard_client.set("/shared", "ActualValue")
        self.assertEqual(cons.get_input("input", default=default_value), "ActualValue")

    def test_get_input_with_default_none_and_no_data(self) -> None:
        """Test that get_input raises an exception when default is None and no value is set."""
        cons = Consumer("cons")
        cons.setup_ports(port_remappings={"input": "/shared"})
        # No value is set on the blackboard for the input port
        with self.assertRaises(NoDataAvailable):
            cons.get_input("input")

    def test_relative_remap_resolves_against_subtree_namespace(self) -> None:
        """
        Two sibling subtrees using the *same* relative remap key must not collide.

        Regression test: a remap like ``{"output": "transfer"}`` under
        ``subtree_namespace="/ns1"`` must resolve to ``/ns1/transfer``, not
        to the global literal key ``"transfer"``. Otherwise sibling subtrees
        silently overwrite each other's data.
        """
        prod_a = Producer("prod_a")
        prod_b = Producer("prod_b")

        prod_a.setup_ports(port_remappings={"output": "transfer"}, subtree_namespace="/ns1")
        prod_b.setup_ports(port_remappings={"output": "transfer"}, subtree_namespace="/ns2")

        # Keys should resolve to their respective subtree namespaces.
        self.assertEqual(prod_a._get_blackboard_key("output"), "/ns1/transfer")
        self.assertEqual(prod_b._get_blackboard_key("output"), "/ns2/transfer")

        # And, critically, the two subtrees must not overwrite each other.
        prod_a._set_output("output", "value_a")
        prod_b._set_output("output", "value_b")
        self.assertEqual(prod_a.get_last_output("output"), "value_a")
        self.assertEqual(prod_b.get_last_output("output"), "value_b")

    def test_relative_remap_wires_siblings_within_same_subtree(self) -> None:
        """A relative remap shared by two nodes in the same namespace wires them together."""
        prod = Producer("prod")
        cons = Consumer("cons")
        prod.setup_ports(port_remappings={"output": "shared"}, subtree_namespace="/ns")
        cons.setup_ports(port_remappings={"input": "shared"}, subtree_namespace="/ns")
        self.assertEqual(prod._get_blackboard_key("output"), "/ns/shared")
        self.assertEqual(cons._get_blackboard_key("input"), "/ns/shared")

        prod._set_output("output", "wired")
        self.assertEqual(cons.get_input("input"), "wired")


class _RegistryLeaf(BehaviourWithPorts, register=False):
    """Concrete leaf used to exercise the registry; itself kept out of it."""

    @classmethod
    def input_ports(cls) -> dict:
        return {}

    @classmethod
    def output_ports(cls) -> dict:
        return {}

    def update(self) -> py_trees.common.Status:
        return py_trees.common.Status.SUCCESS


class TestPortsClassRegistry(unittest.TestCase):
    """Auto-registration of PortsMixin subclasses for tree parsers."""

    def test_concrete_subclass_is_registered(self) -> None:
        # Producer is a concrete BehaviourWithPorts imported by the test helpers.
        self.assertIs(get_ports_registry().get("Producer"), Producer)

    def test_abstract_bases_are_not_registered(self) -> None:
        # Neither the mixin nor the still-abstract convenience base is registered.
        registry = get_ports_registry()
        self.assertNotIn("PortsMixin", registry)
        self.assertNotIn("BehaviourWithPorts", registry)

    def test_tag_alias_at_definition(self) -> None:
        class AliasedLeaf(_RegistryLeaf, tag="AliasTag"):
            pass

        registry = get_ports_registry()
        self.assertIs(registry.get("AliasTag"), AliasedLeaf)
        self.assertNotIn("AliasedLeaf", registry)

    def test_register_false_opts_out(self) -> None:
        class HiddenLeaf(_RegistryLeaf, register=False):
            pass

        self.assertNotIn("HiddenLeaf", get_ports_registry())

    def test_register_ports_class_aliases(self) -> None:
        register_ports_class("ProducerAlias", Producer)
        self.assertIs(get_ports_registry().get("ProducerAlias"), Producer)

    def test_register_ports_class_rejects_non_portsmixin(self) -> None:
        with self.assertRaises(TypeError):
            register_ports_class("NotAPort", py_trees.behaviour.Behaviour)

    def test_duplicate_name_warns_and_last_wins(self) -> None:
        class FirstLeaf(_RegistryLeaf, register=False):
            pass

        class SecondLeaf(_RegistryLeaf, register=False):
            pass

        register_ports_class("DupTag", FirstLeaf)
        with self.assertWarns(UserWarning):
            register_ports_class("DupTag", SecondLeaf)
        self.assertIs(get_ports_registry().get("DupTag"), SecondLeaf)


if __name__ == "__main__":
    unittest.main()

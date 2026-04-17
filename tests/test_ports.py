import unittest

import py_trees

from py_trees.ports import NoDataAvailable

from .test_ports_helpers import Consumer, ConsumerProducer, Producer


# TODO: Add more tests for PortsMixin methods as needed. There are also some tests that can be ported over from
# test_behavior_with_ports.py to here.
class TestPortsMixin(unittest.TestCase):
    def setUp(self):
        self.mixin = Producer("test")  # Use Producer class so we don't have to worry about PortsMixin abstract methods

    def test_basic_types(self):
        self.assertTrue(self.mixin._is_instance_of_type(5, int))
        self.assertTrue(self.mixin._is_instance_of_type(3.14, float))
        self.assertTrue(self.mixin._is_instance_of_type("hello", str))
        self.assertFalse(self.mixin._is_instance_of_type("5", int))
        self.assertFalse(self.mixin._is_instance_of_type(5, str))

    def test_list_of_int(self):
        self.assertTrue(self.mixin._is_instance_of_type([1, 2, 3], list[int]))
        self.assertFalse(self.mixin._is_instance_of_type([1, "2", 3], list[int]))
        self.assertTrue(self.mixin._is_instance_of_type([], list[int]))  # empty list is valid

    def test_union_type(self):
        T = int | str
        self.assertTrue(self.mixin._is_instance_of_type(5, T))
        self.assertTrue(self.mixin._is_instance_of_type("hello", T))
        self.assertFalse(self.mixin._is_instance_of_type(3.14, T))

    def test_list_of_union(self):
        T = list[int | str]
        self.assertTrue(self.mixin._is_instance_of_type([1, "a", 2], T))
        self.assertFalse(self.mixin._is_instance_of_type([1, 2.0], T))

    def test_or_operator(self):
        T = int | str
        self.assertTrue(self.mixin._is_instance_of_type(5, T))
        self.assertTrue(self.mixin._is_instance_of_type("hello", T))
        self.assertFalse(self.mixin._is_instance_of_type(3.14, T))

        T_list = list[int | str]
        self.assertTrue(self.mixin._is_instance_of_type([1, "a", 2], T_list))
        self.assertFalse(self.mixin._is_instance_of_type([1, 2.0], T_list))

    def test_list_or_element(self):
        T = int | list[int]
        self.assertTrue(self.mixin._is_instance_of_type(5, T))
        self.assertTrue(self.mixin._is_instance_of_type([1, 2, 3], T))
        self.assertFalse(self.mixin._is_instance_of_type("hello", T))
        self.assertFalse(self.mixin._is_instance_of_type([1, "2"], T))

    def test_not_implemented_for_dict(self):
        with self.assertRaises(NotImplementedError):
            self.mixin._is_instance_of_type({"a": 1}, dict[str, int])


class TestBehaviourWithPorts(unittest.TestCase):
    def setUp(self):
        """Reset the blackboard before each test."""
        py_trees.blackboard.Blackboard.clear()

    def test_faulty_output_type(self):
        """Test that setting an output with an incorrect type raises a TypeError."""
        cap = Producer("FaultyOutput")
        cap.setup_ports()
        with self.assertRaises(TypeError):
            cap._set_output("output", 123)

    def test_faulty_input_type(self):
        """Test that getting an input with an incorrect type raises a TypeError."""
        cap = Consumer("FaultyInput")
        cap.setup_ports(port_remappings={"input": "/sometest/input"})
        blackboard_client = py_trees.blackboard.Client(name="SomeoneElse")
        blackboard_client.register_key(key="/sometest/input", access=py_trees.common.Access.WRITE, required=True)
        blackboard_client.set("/sometest/input", 123)
        with self.assertRaises(TypeError):
            cap.get_input("input")

    def test_simple_remapping(self):
        """Test that a simple port remapping between producer and consumer works as expected."""
        # Producer out -> /shared, Consumer in -> /shared
        prod = Producer("prod")
        cons = Consumer("cons")
        prod.setup_ports(port_remappings={"output": "/shared"})
        cons.setup_ports(port_remappings={"input": "/shared"})
        prod._set_output("output", "HelloWorld")
        self.assertEqual(cons.get_input("input"), "HelloWorld")

    def test_default_ports_are_unique_per_node(self):
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

    def test_multilevel_remapping(self):
        """Test that multi-level port remapping through nested subtrees propagates values correctly."""
        # Producer out -> /shared, Consumer in -> /shared
        prod = Producer("prod")
        prod.setup_ports(port_remappings={"output": "/root"}, subtree_namespace="/")
        # Set up a subtree with 2 ConsumerProducers
        sbtr1_consprod1 = ConsumerProducer("sbtr1_consprod1")
        sbtr1_consprod2 = ConsumerProducer("sbtr1_consprod2")
        sbtr1_consprod1.setup_ports(
            port_remappings={"input": "/root", "output": "transfer"}, subtree_namespace="/subtree1"
        )
        sbtr1_consprod2.setup_ports(
            port_remappings={"input": "transfer", "output": "subtree1_output"},
            subtree_namespace="/subtree1",
        )

        # Set up another nested subtree (grandchild) with 2 ConsumerProducers
        sbtr2_consprod1 = ConsumerProducer("sbtr2_consprod1")
        sbtr2_consprod2 = ConsumerProducer("sbtr2_consprod2")
        sbtr2_consprod1.setup_ports(
            port_remappings={"input": "subtree1_output", "output": "transfer"}, subtree_namespace="/subtree1/subtree2"
        )
        sbtr2_consprod2.setup_ports(
            port_remappings={"input": "transfer", "output": "/result"}, subtree_namespace="/subtree1/subtree2"
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

    def test_type_checking(self):
        """Test that type checking is enforced when setting output values."""
        prod = Producer("prod")
        cons = Consumer("cons")
        prod.setup_ports(port_remappings={"output": "/shared"})
        cons.setup_ports(port_remappings={"input": "/shared"})
        with self.assertRaises(TypeError):
            prod._set_output("output", 123)

    def test_subtree_namespace_remapping(self):
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

    def test_get_input_with_default(self):
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

    def test_get_input_with_default_none_and_no_data(self):
        """Test that get_input raises an exception when default is None and no value is set."""
        cons = Consumer("cons")
        cons.setup_ports(port_remappings={"input": "/shared"})
        # No value is set on the blackboard for the input port
        with self.assertRaises(NoDataAvailable):
            cons.get_input("input")


if __name__ == "__main__":
    unittest.main()

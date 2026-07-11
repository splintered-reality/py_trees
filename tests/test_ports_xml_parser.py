#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/splintered-reality/py_trees/devel/LICENSE
#

##############################################################################
# Imports
##############################################################################

# ruff: noqa: SIM115
import os
import tempfile
import time
import unittest
from dataclasses import dataclass
from functools import partial
from typing import Any

import py_trees
from py_trees.parsers.behaviour_tree_xml import is_key, parse_behaviour_tree_xml
from py_trees.ports import BehaviourWithPorts, PortInformation, PortsMixin, get_ports_registry
from py_trees.ports_utils import (
    find_node_by_class,
    find_node_by_name,
    generate_node_name,
    strip_trailing_uuid4,
)

from .test_ports_helpers import Consumer, Producer


class StdoutLogger:
    """Simple stdout logger for test debug output (replaces the removed standalone StdoutLogger class)."""

    def info(self, msg: str) -> None:
        print(f"[INFO] {msg}")

    def warning(self, msg: str) -> None:
        print(f"[WARNING] {msg}")

    def error(self, msg: str) -> None:
        print(f"[ERROR] {msg}")

    def debug(self, msg: str) -> None:
        print(f"[DEBUG] {msg}")


class DummyFactory:
    pass


class Wait(BehaviourWithPorts):
    INPUT_DURATION_MS_PORT = "input_duration_ms"

    def __init__(self, name: str, factory: DummyFactory, **kwargs: Any) -> None:
        super().__init__(name=name, **kwargs)
        self._factory = factory
        self.start_time = 0.0

    @classmethod
    def input_ports(cls) -> dict:
        return {cls.INPUT_DURATION_MS_PORT: PortInformation(data_type=int, required=True)}

    @classmethod
    def output_ports(cls) -> dict:
        return {}

    def initialise(self) -> None:
        self.start_time = time.time()

    def update(self) -> py_trees.common.Status:
        if self.duration_value_ms < 0:
            return py_trees.common.Status.RUNNING
        return (
            py_trees.common.Status.SUCCESS
            if (time.time() - self.start_time) >= self.duration_value_ms / 1000.0
            else py_trees.common.Status.RUNNING
        )

    @property
    def duration_value_ms(self) -> Any:
        return self.get_input(self.INPUT_DURATION_MS_PORT)


class EchoCtorArgs(BehaviourWithPorts):
    """Behaviour that tests interpreting constructor type hints for type coercion from XML ports."""

    @classmethod
    def input_ports(cls) -> dict:
        return {"in": PortInformation(data_type=str, required=False)}  # not used here

    @classmethod
    def output_ports(cls) -> dict:
        return {"out": PortInformation(data_type=str, required=False)}  # not used here

    def __init__(self, name: str, greeting: str, times: float | None, flag: bool, **kwargs: Any) -> None:
        super().__init__(name, **kwargs)
        self.greeting = greeting
        self.times = times
        self.flag = flag


class EchoCtorArgsChild(EchoCtorArgs):
    """
    Behaviour that tests interpreting constructor type hints for type coercion from XML ports,
    but from its parent class.
    """

    def __init__(self, **kwargs):
        super().__init__(**kwargs)


@dataclass
class RobotData:
    type: str
    ordered_joint_names: list[str]
    commander: object | None


def get_behaviors_lookup(factory: DummyFactory, _robot_data: dict[int, RobotData]) -> dict:
    return {"Wait": partial(Wait, factory=factory)}


class TestXMLParser(unittest.TestCase):
    # Find the final consumer node

    def setUp(self) -> None:
        py_trees.blackboard.Blackboard.clear()
        # Minimal XML with remapping and a subtree
        self.xml = """<root main_tree_to_execute="MainTree">
        <BehaviorTree ID="SubTree">
          <Sequence>
            <Producer name="prod" output="{subtree_out}" />
          </Sequence>
        </BehaviorTree>
        <BehaviorTree ID="MainTree">
          <Sequence>
            <SubTree ID="SubTree" name="SubTree" subtree_out="{final}" />
            <Consumer name="cons" input="{final}" />
          </Sequence>
        </BehaviorTree>
      </root>"""
        self.tempfile = tempfile.NamedTemporaryFile(delete=False, mode="w", suffix=".xml")
        self.tempfile.write(self.xml)
        self.tempfile.close()
        self.factory = DummyFactory()

    def tearDown(self) -> None:
        os.unlink(self.tempfile.name)

    def test_is_key_rejects_malformed_braces(self) -> None:
        self.assertIsNone(is_key("/absolute/path"))
        self.assertIsNone(is_key("relative_value"))
        match = is_key("{logical_key}")
        assert match is not None
        self.assertEqual(match.group(1), "logical_key")

        malformed_values = [
            "{missing_close",
            "missing_open}",
            "{{nested}}",
            "{first}{second}",
            "prefix{key}",
            "{key}suffix",
        ]
        for value in malformed_values:
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    is_key(value)

    def test_xml_parser_remapping(self) -> None:
        """Ensure remapping between subtrees works correctly."""
        root_node = parse_behaviour_tree_xml(self.tempfile.name, logger=StdoutLogger())
        # Wrap in a py_trees BehaviourTree and tick until complete
        btree = py_trees.trees.BehaviourTree(root_node)
        btree.tick()
        cons = find_node_by_class(btree.root, Consumer)
        self.assertIsNotNone(cons)
        self.assertEqual(cons.consumed_value, "Producer[/SubTree:SubTree.prod]")

    def test_grandparent_xml(self) -> None:
        """Test XML parser with grandparent-child relationships and correct value propagation."""
        xml_path = os.path.join(os.path.dirname(__file__), "xml", "grandparent_test.xml")
        root_node = parse_behaviour_tree_xml(xml_path, logger=StdoutLogger())
        btree = py_trees.trees.BehaviourTree(root_node)
        btree.tick()
        cons = find_node_by_name(btree.root, generate_node_name("ConsumerMain"))
        assert isinstance(cons, Consumer)
        expected_value = (
            "Producer"
            "[/:ProducerMain]"
            "[/SubTreeMain1/SubTree1:SubTreeMain1.SubTree1.ConsumerProducer1]"
            "[/SubTreeMain1/SubTree1:SubTreeMain1.SubTree1.ConsumerProducer2]"
            "[/SubTreeMain1/SubTree2:SubTreeMain1.SubTree2.ConsumerProducer1]"
            "[/SubTreeMain1/SubTree2:SubTreeMain1.SubTree2.ConsumerProducer2]"
            "[/SubTreeMain2/SubTree1:SubTreeMain2.SubTree1.ConsumerProducer1]"
            "[/SubTreeMain2/SubTree1:SubTreeMain2.SubTree1.ConsumerProducer2]"
            "[/SubTreeMain2/SubTree2:SubTreeMain2.SubTree2.ConsumerProducer1]"
            "[/SubTreeMain2/SubTree2:SubTreeMain2.SubTree2.ConsumerProducer2]"
        )
        self.assertEqual(cons.consumed_value, expected_value)

    def test_custom_behavior_with_extra_arg(self) -> None:
        """Test custom behavior with an additional argument."""

        class CustomBehaviourWithPorts(BehaviourWithPorts):
            @classmethod
            def input_ports(cls) -> dict:
                return {"in": PortInformation(data_type=str, required=False)}

            @classmethod
            def output_ports(cls) -> dict:
                return {"out": PortInformation(data_type=str, required=False)}

            def __init__(self, name: str, extra_arg: str, **kwargs: Any) -> None:
                super().__init__(name, **kwargs)
                self.extra_arg = extra_arg

        # Minimal XML for the custom behavior
        xml = """<root main_tree_to_execute="MainTree">
          <BehaviorTree ID="MainTree">
            <Sequence>
              <CustomBehaviourWithPorts name="custom1" />
            </Sequence>
          </BehaviorTree>
        </root>"""
        with tempfile.NamedTemporaryFile(delete=False, mode="w", suffix=".xml") as tf:
            tf.write(xml)
            temp_xml_path = tf.name

        # Other classes auto-register; the partial injects the extra constructor arg.
        custom_lookup = {"CustomBehaviourWithPorts": partial(CustomBehaviourWithPorts, extra_arg="hello-world")}

        try:
            root_node = parse_behaviour_tree_xml(temp_xml_path, node_registry=custom_lookup, logger=StdoutLogger())
            custom = find_node_by_class(root_node, CustomBehaviourWithPorts)
            self.assertIsNotNone(custom)
            self.assertEqual(custom.extra_arg, "hello-world")
        finally:
            os.unlink(temp_xml_path)

    def test_subtree_remapping_only_explicit_keys(self) -> None:
        """Check that only explicitly remapped keys are used in subtrees."""
        xml_content = """
        <root main_tree_to_execute="MainTree">
            <BehaviorTree ID="MySubtree">
                <Sequence>
                    <Consumer name="MyConsumer" input="{input_key}" />
                    <Producer name="MyInternalProducer" output="{transfer_key}" />
                    <Consumer name="MyInternalConsumer" input="{transfer_key}" />
                </Sequence>
            </BehaviorTree>
            <BehaviorTree ID="MainTree">
                <Sequence>
                    <Producer output="{some_key}" name="ProducerMain1" />
                    <Producer output="{transfer_key}" name="ProducerMain2" />
                    <SubTree ID="MySubtree" name="Subtree1" input_key="{some_key}"/>
                </Sequence>
            </BehaviorTree>
        </root>
        """
        with tempfile.NamedTemporaryFile(delete=False, mode="w", suffix=".xml") as tf:
            tf.write(xml_content)
            temp_xml_path = tf.name

        root = parse_behaviour_tree_xml(temp_xml_path)
        btree = py_trees.trees.BehaviourTree(root)
        btree.tick()

        # MyInternalConsumer does not get the {transfer_key} in the main namespace
        node = find_node_by_name(root, "MyInternalConsumer", strip_prefix=True)
        assert isinstance(node, Consumer)

        expected_value = "Producer[/Subtree1:Subtree1.MyInternalProducer]"
        self.assertEqual(node.consumed_value, expected_value)

    def test_tree_structure(self) -> None:
        """Verify that the structure of the behavior tree matches the XML."""
        root_node = parse_behaviour_tree_xml(self.tempfile.name, logger=StdoutLogger())
        # Check root node is a Sequence
        self.assertIsInstance(root_node, py_trees.composites.Sequence)
        self.assertEqual(len(root_node.children), 2)

        # assert that the first child is the root of the subtree
        self.assertEqual(len(root_node.children), 2)
        subtree = root_node.children[0]
        self.assertEqual(strip_trailing_uuid4(subtree.name), "SubTree.Sequence")
        self.assertIsInstance(subtree, py_trees.behaviour.Behaviour)

        # Check second child is a Consumer
        consumer = root_node.children[1]
        self.assertEqual(consumer.name, "cons")
        self.assertIsInstance(consumer, Consumer)

        # Check SubTree structure: has only one child (the Producer).
        self.assertEqual(len(subtree.children), 1)
        producer = subtree.children[0]
        self.assertEqual(producer.name, "SubTree.prod")
        self.assertIsInstance(producer, Producer)

    def test_parsing_simple_direct_values_to_XML(self) -> None:
        """Verify that simple direct values are successfully parsed via the XML parser."""
        # Minimal XML with direct values and a subtree
        self.xml = """<root main_tree_to_execute="MainTree">
        <BehaviorTree ID="MainTree">
          <Sequence>
            <Consumer name="cons" input="ABC" />
          </Sequence>
        </BehaviorTree>
        </root>"""

        self.tempfile = tempfile.NamedTemporaryFile(delete=False, mode="w", suffix=".xml")
        self.tempfile.write(self.xml)
        self.tempfile.close()

        root_node = parse_behaviour_tree_xml(self.tempfile.name, logger=StdoutLogger())
        btree = py_trees.trees.BehaviourTree(root_node)

        btree.tick()

        node = find_node_by_name(root_node, "cons")
        assert isinstance(node, Consumer)

        self.assertEqual(type(node.consumed_value), str)
        self.assertEqual(node.consumed_value, "ABC")

    def test_parsing_direct_values_to_XML(self) -> None:
        """Verify that direct values are successfully parsed via the XML parser."""
        # Minimal XML with direct values and a subtree
        self.xml = """<root main_tree_to_execute="MainTree">
        <BehaviorTree ID="SubTree">
          <Sequence>
            <Producer name="prod" output="{subtree_out}" />
            <Consumer name="internalcons" input="{subtree_out}" />
          </Sequence>
        </BehaviorTree>

        <BehaviorTree ID="MainTree">
          <Sequence>
            <SubTree ID="SubTree" name="Subtree1" subtree_out="{final}" />
            <Consumer name="cons" input="500" />
          </Sequence>
        </BehaviorTree>
        </root>"""

        self.tempfile = tempfile.NamedTemporaryFile(delete=False, mode="w", suffix=".xml")
        self.tempfile.write(self.xml)
        self.tempfile.close()

        root_node = parse_behaviour_tree_xml(self.tempfile.name, logger=StdoutLogger())
        btree = py_trees.trees.BehaviourTree(root_node)

        btree.tick()

        node = find_node_by_name(root_node, "internalcons", strip_prefix=True)
        assert isinstance(node, Consumer)

        expected_value = "Producer[/Subtree1:Subtree1.prod]"
        self.assertEqual(node.consumed_value, expected_value)

        node = find_node_by_name(root_node, "cons", strip_prefix=True)
        assert isinstance(node, Consumer)

        expected_value = "500"

        self.assertEqual(node.consumed_value, expected_value)

    def test_subtree_parsing_direct_values_to_XML(self) -> None:
        """Verify that direct values are successfully parsed via the XML parser."""
        # Minimal XML with direct values and a subtree
        self.xml = """<root main_tree_to_execute="MainTree">
        <BehaviorTree ID="SubTree">
          <Sequence>
            <Consumer name="Consumer1" input="sunrise"/>
            <Producer name="Producer1" output="{subtree_tmp}"/>
            <Consumer name="Consumer2" input="100"/>
            <Producer name="Producer2" output="{subtree_in}"/>
            <Consumer name="InternalConsumer3" input="{subtree_in}"/>
          </Sequence>
        </BehaviorTree>

        <BehaviorTree ID="MainTree">
          <Sequence>
            <Producer output="{initial_value}" name="ProducerMain" />
            <SubTree ID="SubTree" name="SubTree1" subtree_in="{initial_value}" subtree_out="{final}" />
            <Consumer name="ConsumerMain" input="{final}" />
          </Sequence>
        </BehaviorTree>
        </root>"""

        self.tempfile = tempfile.NamedTemporaryFile(delete=False, mode="w", suffix=".xml")
        self.tempfile.write(self.xml)
        self.tempfile.close()

        root_node = parse_behaviour_tree_xml(self.tempfile.name, logger=StdoutLogger())
        btree = py_trees.trees.BehaviourTree(root_node)

        btree.tick()

        node = find_node_by_name(root_node, "InternalConsumer3", strip_prefix=True)
        assert isinstance(node, Consumer)

        expected_value = "Producer[/SubTree1:SubTree1.Producer2]"
        self.assertEqual(node.consumed_value, expected_value)

        node = find_node_by_name(root_node, "Consumer1", strip_prefix=True)
        assert isinstance(node, Consumer)

        expected_value = "sunrise"

        self.assertEqual(node.consumed_value, expected_value)

        node = find_node_by_name(root_node, "Consumer2", strip_prefix=True)
        assert isinstance(node, Consumer)

        expected_value = "100"

        self.assertEqual(node.consumed_value, expected_value)

    def test_wait_node(self) -> None:
        """Verify that the duration value is successfully used by the Wait node."""
        wait_duration_ms = 500
        # Minimal XML using the Wait behavior.
        self.xml = f"""<root main_tree_to_execute="MainTree">
        <BehaviorTree ID="MainTree">
          <Sequence>
            <Wait name="Pause1" input_duration_ms="{wait_duration_ms}"/>
          </Sequence>
        </BehaviorTree>
        </root>"""

        self.tempfile = tempfile.NamedTemporaryFile(delete=False, mode="w", suffix=".xml")
        self.tempfile.write(self.xml)
        self.tempfile.close()

        node_registry = {"Wait": partial(Wait, factory=self.factory)}

        root_node = parse_behaviour_tree_xml(self.tempfile.name, node_registry=node_registry, logger=StdoutLogger())
        btree = py_trees.trees.BehaviourTree(root_node)

        start_time = time.time()
        while btree.root.status != py_trees.common.Status.SUCCESS:
            btree.tick()
            time.sleep(0.01)
        duration = time.time() - start_time

        node = find_node_by_name(root_node, "Pause1", strip_prefix=True)
        assert isinstance(node, Wait)

        self.assertEqual(node.duration_value_ms, wait_duration_ms)
        self.assertAlmostEqual(duration, wait_duration_ms / 1000.0, delta=1)

    def test_wait_with_registry_node(self) -> None:
        """Verify that the duration value is successfully used by the Wait node using the registry."""
        wait_duration_ms = 2000
        # Minimal XML using the Wait behavior.
        self.xml = f"""<root main_tree_to_execute="MainTree">
        <BehaviorTree ID="MainTree">
          <Sequence>
            <Wait name="Pause1" input_duration_ms="{wait_duration_ms}"/>
          </Sequence>
        </BehaviorTree>
        </root>"""

        self.tempfile = tempfile.NamedTemporaryFile(delete=False, mode="w", suffix=".xml")
        self.tempfile.write(self.xml)
        self.tempfile.close()

        # Getting from the registry
        SMALL_JOINT_NAMES = [f"robot_small_joint_{i}" for i in range(1, 7)]
        BIG_JOINT_NAMES = [f"robot_big_joint_{i}" for i in range(1, 7)]
        node_registry = get_behaviors_lookup(
            self.factory,
            {
                0: RobotData("robot_small", SMALL_JOINT_NAMES, None),
                1: RobotData("robot_big", BIG_JOINT_NAMES, None),
            },
        )

        root_node = parse_behaviour_tree_xml(self.tempfile.name, node_registry=node_registry, logger=StdoutLogger())
        btree = py_trees.trees.BehaviourTree(root_node)

        start_time = time.time()
        while btree.root.status != py_trees.common.Status.SUCCESS:
            btree.tick()
            time.sleep(0.01)
        duration = time.time() - start_time

        node = find_node_by_name(root_node, "Pause1", strip_prefix=True)
        assert isinstance(node, Wait)

        self.assertEqual(node.duration_value_ms, wait_duration_ms)
        self.assertAlmostEqual(duration, wait_duration_ms / 1000.0, delta=1)

    def test_ctor_args_passed_as_kwargs(self) -> None:
        """
        Non-port XML attributes must be passed as constructor kwargs.
        Values are automatically converted based on the type hints in the constructor.
        """

        xml = """<root main_tree_to_execute="Main">
        <BehaviorTree ID="Main">
          <Sequence>
            <EchoCtorArgs name="E1" greeting="hello" times="3" flag="true"/>
          </Sequence>
        </BehaviorTree>
      </root>"""

        with tempfile.NamedTemporaryFile(delete=False, mode="w", suffix=".xml") as tf:
            tf.write(xml)
            path = tf.name

        root = parse_behaviour_tree_xml(path, logger=StdoutLogger())
        node = find_node_by_class(root, EchoCtorArgs)
        self.assertIsNotNone(node)
        self.assertEqual(node.greeting, "hello")
        self.assertEqual(node.times, 3.0)
        self.assertEqual(node.flag, True)

        os.unlink(path)

    def test_ctor_args_passed_as_kwargs_in_parent_class(self) -> None:
        """
        Non-port XML attributes must be passed as constructor kwargs.
        Values are automatically converted based on the type hints in the parent class constructor.
        """

        xml = """<root main_tree_to_execute="Main">
        <BehaviorTree ID="Main">
          <Sequence>
            <EchoCtorArgsChild name="E1" greeting="hello" times="3" flag="true"/>
          </Sequence>
        </BehaviorTree>
      </root>"""

        with tempfile.NamedTemporaryFile(delete=False, mode="w", suffix=".xml") as tf:
            tf.write(xml)
            path = tf.name

        root = parse_behaviour_tree_xml(path, logger=StdoutLogger())
        node = find_node_by_class(root, EchoCtorArgsChild)
        self.assertIsNotNone(node)
        self.assertEqual(node.greeting, "hello")
        self.assertEqual(node.times, 3.0)
        self.assertEqual(node.flag, True)

        os.unlink(path)

    def test_mixed_ports_and_ctor_kwargs(self) -> None:
        """
        If an attribute matches a declared port, it is handled as a port (remap/resolve)
        and must NOT be passed as a constructor kwarg. Non-port attributes become ctor kwargs.
        """

        class PortAndCtor(BehaviourWithPorts):
            @classmethod
            def input_ports(cls) -> dict:
                return {"in": PortInformation(data_type=str, required=True)}  # only this is a port

            @classmethod
            def output_ports(cls) -> dict:
                return {"out": PortInformation(data_type=str, required=False)}

            def __init__(self, name: str, label: str, **kwargs: Any) -> None:
                super().__init__(name, **kwargs)
                self.label = label
                self.in_value: Any = None

            def update(self) -> py_trees.common.Status:
                self.in_value = self.get_input("in")
                return py_trees.common.Status.SUCCESS

        xml = """<root main_tree_to_execute="Main">
          <BehaviorTree ID="Main">
            <Sequence>
              <PortAndCtor name="P" in="CONST" label="LBL"/>
            </Sequence>
          </BehaviorTree>
        </root>"""

        with tempfile.NamedTemporaryFile(delete=False, mode="w", suffix=".xml") as tf:
            tf.write(xml)
            path = tf.name

        try:
            root = parse_behaviour_tree_xml(path, logger=StdoutLogger())
            tree = py_trees.trees.BehaviourTree(root)
            tree.tick()

            node = find_node_by_class(root, PortAndCtor)
            self.assertIsNotNone(node)

            # Assert constructor-only arg is set
            self.assertEqual(node.label, "LBL")

            # Assert port input came via blackboard (direct value path)
            self.assertEqual(node.in_value, "CONST")
        finally:
            os.unlink(path)

    def test_ctor_arg_with_curly_value_is_not_resolved(self) -> None:
        """
        Non-port attributes that look like keys (e.g., "{foo}") raise an exception.
        """

        class TakesKeyString(BehaviourWithPorts):
            @classmethod
            def input_ports(cls) -> dict:
                return {}  # no ports at all

            @classmethod
            def output_ports(cls) -> dict:
                return {}

            def __init__(self, name: str, token: str) -> None:
                super().__init__(name)
                self.token = token

        xml = """<root main_tree_to_execute="Main">
          <BehaviorTree ID="Main">
            <Sequence>
              <TakesKeyString name="T" token="{abc}"/>
            </Sequence>
          </BehaviorTree>
        </root>"""

        with tempfile.NamedTemporaryFile(delete=False, mode="w", suffix=".xml") as tf:
            tf.write(xml)
            path = tf.name

        with self.assertRaises(ValueError):
            parse_behaviour_tree_xml(path, logger=StdoutLogger())
        os.unlink(path)

    def test_decorator_node(self) -> None:
        """Verify that a built-in decorator is instantiated with typed constructor kwargs."""
        xml = """<root main_tree_to_execute="MainTree">
          <BehaviorTree ID="MainTree">
            <Sequence>
              <Repeat num_success="-1">
                <Producer name="prod" output="{out}" />
              </Repeat>
            </Sequence>
          </BehaviorTree>
        </root>"""

        with tempfile.NamedTemporaryFile(delete=False, mode="w", suffix=".xml") as tf:
            tf.write(xml)
            path = tf.name

        try:
            root = parse_behaviour_tree_xml(path, logger=StdoutLogger())
            repeat = find_node_by_class(root, py_trees.decorators.Repeat)
            self.assertIsNotNone(repeat)
            self.assertEqual(repeat.num_success, -1)
            self.assertIsInstance(repeat.num_success, int)
            self.assertIsInstance(repeat.decorated, Producer)

            # num_success=-1 repeats indefinitely, so the decorator stays RUNNING.
            btree = py_trees.trees.BehaviourTree(root)
            btree.tick()
            self.assertEqual(repeat.status, py_trees.common.Status.RUNNING)
        finally:
            os.unlink(path)

    def test_decorator_ports_node(self) -> None:
        """Verify that a decorator with ports is instantiated correctly."""

        class RepeatWithPorts(PortsMixin, py_trees.decorators.Repeat):
            """A `py_trees.decorators.Repeat` decorator that also exposes ports."""

            @classmethod
            def input_ports(cls) -> dict:
                return {}

            @classmethod
            def output_ports(cls) -> dict:
                return {"count": PortInformation(data_type=int, required=False)}

        xml = """<root main_tree_to_execute="MainTree">
          <BehaviorTree ID="MainTree">
            <Sequence>
              <RepeatWithPorts num_success="5">
                <Producer name="prod" output="{out}" />
              </RepeatWithPorts>
            </Sequence>
          </BehaviorTree>
        </root>"""

        with tempfile.NamedTemporaryFile(delete=False, mode="w", suffix=".xml") as tf:
            tf.write(xml)
            path = tf.name

        try:
            root = parse_behaviour_tree_xml(path, logger=StdoutLogger())
            repeat = find_node_by_class(root, RepeatWithPorts)
            self.assertIsNotNone(repeat)
            self.assertEqual(repeat.num_success, 5)
            self.assertIsInstance(repeat.num_success, int)
            self.assertIsInstance(repeat.decorated, Producer)

            # num_success=-1 repeats indefinitely, so the decorator stays RUNNING.
            btree = py_trees.trees.BehaviourTree(root)
            btree.tick()
            self.assertEqual(repeat.status, py_trees.common.Status.RUNNING)
        finally:
            os.unlink(path)

    def test_composite_ports_node(self) -> None:
        """Verify that a decorator with ports is instantiated correctly."""

        class SequenceWithPorts(PortsMixin, py_trees.composites.Sequence):
            """A `py_trees.composites.Sequence` composite that also exposes ports."""

            @classmethod
            def input_ports(cls) -> dict:
                return {}

            @classmethod
            def output_ports(cls) -> dict:
                return {}

        xml = """<root main_tree_to_execute="MainTree">
          <BehaviorTree ID="MainTree">
            <SequenceWithPorts memory="true">
                <Producer name="prod1" output="{out1}" />
                <Producer name="prod2" output="{out2}" />
            </SequenceWithPorts>
          </BehaviorTree>
        </root>"""

        with tempfile.NamedTemporaryFile(delete=False, mode="w", suffix=".xml") as tf:
            tf.write(xml)
            path = tf.name

        try:
            root = parse_behaviour_tree_xml(path, logger=StdoutLogger())
            sequence = find_node_by_class(root, SequenceWithPorts)
            self.assertIsNotNone(sequence)
            self.assertIsInstance(sequence.children, list)
            self.assertEqual(len(sequence.children), 2)
            self.assertIsInstance(sequence.children[0], Producer)
            self.assertIsInstance(sequence.children[1], Producer)

            btree = py_trees.trees.BehaviourTree(root)
            btree.tick()
            self.assertEqual(sequence.status, py_trees.common.Status.SUCCESS)
        finally:
            os.unlink(path)

    def test_direct_portsmixin_leaf_accepted(self) -> None:
        """
        A leaf class that is ``PortsMixin + Behaviour`` (not via ``BehaviourWithPorts``)
        should be parsed successfully.

        Regression test: the parser previously rejected anything that wasn't
        specifically an instance of ``BehaviourWithPorts``. The gate has been
        relaxed to accept any ``PortsMixin`` + ``py_trees.behaviour.Behaviour``.
        """
        from py_trees.ports import PortsMixin

        class DirectPortsLeaf(PortsMixin, py_trees.behaviour.Behaviour):
            """PortsMixin leaf that does NOT go through BehaviourWithPorts."""

            def __init__(self, name: str, **kwargs: Any) -> None:
                super().__init__(name=name, **kwargs)

            @classmethod
            def input_ports(cls) -> dict:
                return {}

            @classmethod
            def output_ports(cls) -> dict:
                return {"out": PortInformation(data_type=str, required=True)}

            def update(self) -> py_trees.common.Status:
                self._set_output("out", "direct-ports-leaf-ran")
                return py_trees.common.Status.SUCCESS

        # Sanity: not a BehaviourWithPorts, but is a PortsMixin + Behaviour.
        from py_trees.ports import BehaviourWithPorts

        self.assertFalse(issubclass(DirectPortsLeaf, BehaviourWithPorts))
        self.assertTrue(issubclass(DirectPortsLeaf, PortsMixin))
        self.assertTrue(issubclass(DirectPortsLeaf, py_trees.behaviour.Behaviour))

        xml = """<root main_tree_to_execute="Main">
          <BehaviorTree ID="Main">
            <Sequence>
              <DirectPortsLeaf name="direct" out="{result}"/>
            </Sequence>
          </BehaviorTree>
        </root>"""
        with tempfile.NamedTemporaryFile(delete=False, mode="w", suffix=".xml") as tf:
            tf.write(xml)
            path = tf.name

        try:
            root = parse_behaviour_tree_xml(path, logger=StdoutLogger())
            tree = py_trees.trees.BehaviourTree(root)
            tree.tick()
            leaf = find_node_by_class(root, DirectPortsLeaf)
            self.assertIsNotNone(leaf)
            self.assertEqual(leaf.get_last_output("out"), "direct-ports-leaf-ran")
        finally:
            os.unlink(path)


class TestXMLParserImports(unittest.TestCase):
    """Tests for XML import pre-processing (top-level <Import>/<Include>)."""

    def setUp(self) -> None:
        py_trees.blackboard.Blackboard.clear()

    def _write_temp_xml(self, content: str) -> str:
        tf = tempfile.NamedTemporaryFile(delete=False, mode="w", suffix=".xml")
        tf.write(content)
        tf.close()
        return tf.name

    def tearDown(self) -> None:
        # Nothing to do; each test cleans its own temps
        pass

    def test_import_basic(self) -> None:
        """Main references a subtree defined in an imported file."""
        lib_xml = """<root>
          <BehaviorTree ID="LibTree">
            <Sequence>
              <Producer name="lib_prod" output="{out}"/>
            </Sequence>
          </BehaviorTree>
        </root>"""
        lib_path = self._write_temp_xml(lib_xml)

        main_xml = f"""<root main_tree_to_execute="Main">
          <Include src="{lib_path}"/>
          <BehaviorTree ID="Main">
            <Sequence name="MYSEQ">
              <SubTree ID="LibTree" name="Lib1" out="{{final}}"/>
              <Consumer name="C" input="{{final}}"/>
            </Sequence>
          </BehaviorTree>
        </root>"""
        main_path = self._write_temp_xml(main_xml)

        try:
            root = parse_behaviour_tree_xml(main_path, logger=StdoutLogger())
            tree = py_trees.trees.BehaviourTree(root)
            tree.tick()
            c = find_node_by_name(root, "C", strip_prefix=True)
            assert isinstance(c, Consumer)
            self.assertEqual(c.consumed_value, "Producer[/Lib1:MYSEQ.Lib1.lib_prod]")
        finally:
            os.unlink(lib_path)
            os.unlink(main_path)

    def test_import_duplicate_id_conflict_with_local(self) -> None:
        """Duplicate BehaviorTree ID between main file and imported file raises ValueError."""
        lib_xml = """<root>
          <BehaviorTree ID="DupTree"><Sequence/></BehaviorTree>
        </root>"""
        lib_path = self._write_temp_xml(lib_xml)

        main_xml = f"""<root main_tree_to_execute="Main">
          <Include src="{lib_path}"/>
          <BehaviorTree ID="DupTree"><Sequence/></BehaviorTree>
          <BehaviorTree ID="Main"><Sequence/></BehaviorTree>
        </root>"""
        main_path = self._write_temp_xml(main_xml)

        try:
            with self.assertRaises(ValueError):
                parse_behaviour_tree_xml(main_path, logger=StdoutLogger())
        finally:
            os.unlink(lib_path)
            os.unlink(main_path)

    def test_import_duplicate_id_conflict_between_imports(self) -> None:
        """Duplicate BehaviorTree ID across two imported files raises ValueError."""
        lib_a = """<root><BehaviorTree ID="SameID"><Sequence/></BehaviorTree></root>"""
        lib_b = """<root><BehaviorTree ID="SameID"><Sequence/></BehaviorTree></root>"""
        path_a = self._write_temp_xml(lib_a)
        path_b = self._write_temp_xml(lib_b)

        main_xml = f"""<root main_tree_to_execute="Main">
          <Include src="{path_a}"/>
          <Include src="{path_b}"/>
          <BehaviorTree ID="Main"><Sequence/></BehaviorTree>
        </root>"""
        main_path = self._write_temp_xml(main_xml)

        try:
            with self.assertRaises(ValueError):
                parse_behaviour_tree_xml(main_path, logger=StdoutLogger())
        finally:
            os.unlink(path_a)
            os.unlink(path_b)
            os.unlink(main_path)

    def test_import_missing_file(self) -> None:
        """Missing import target raises FileNotFoundError."""
        main_xml = """<root main_tree_to_execute="Main">
          <Import src="/does/not/exist/lib.xml"/>
          <BehaviorTree ID="Main"><Sequence/></BehaviorTree>
        </root>"""
        main_path = self._write_temp_xml(main_xml)
        try:
            with self.assertRaises(FileNotFoundError):
                parse_behaviour_tree_xml(main_path, logger=StdoutLogger())
        finally:
            os.unlink(main_path)

    def test_import_main_tree_from_import(self) -> None:
        """main_tree_to_execute can point to a BehaviorTree defined in an imported file."""
        lib_xml = """<root>
          <BehaviorTree ID="ExternalMain">
            <Sequence>
              <Consumer name="X" input="OK"/>
            </Sequence>
          </BehaviorTree>
        </root>"""
        lib_path = self._write_temp_xml(lib_xml)

        main_xml = f"""<root main_tree_to_execute="ExternalMain">
          <Include src="{lib_path}"/>
        </root>"""
        main_path = self._write_temp_xml(main_xml)

        try:
            root = parse_behaviour_tree_xml(main_path, logger=StdoutLogger())
            tree = py_trees.trees.BehaviourTree(root)
            tree.tick()
            x = find_node_by_name(root, "X", strip_prefix=True)
            assert isinstance(x, Consumer)
            self.assertEqual(x.consumed_value, "OK")
        finally:
            os.unlink(lib_path)
            os.unlink(main_path)

    def test_import_nested_not_supported(self) -> None:
        """Nested <Import> inside a BehaviorTree is not supported and causes a parse error."""
        lib_xml = """<root>
          <BehaviorTree ID="Lib"><Sequence/></BehaviorTree>
        </root>"""
        lib_path = self._write_temp_xml(lib_xml)

        # Import is *not* a top-level child; pre-pass ignores it.
        # Later, the XML parser encounters the <Import> as a node and should error.
        main_xml = f"""<root main_tree_to_execute="Main">
          <BehaviorTree ID="Main">
            <Sequence>
              <Import src="{lib_path}"/>
            </Sequence>
          </BehaviorTree>
        </root>"""
        main_path = self._write_temp_xml(main_xml)

        try:
            with self.assertRaises(ValueError):
                parse_behaviour_tree_xml(main_path, logger=StdoutLogger())
        finally:
            os.unlink(lib_path)
            os.unlink(main_path)

    def test_import_with_search_paths(self) -> None:
        """Import path can be resolved via the 'search_paths' argument."""
        with tempfile.TemporaryDirectory() as d_main, tempfile.TemporaryDirectory() as d_lib:
            lib_path = os.path.join(d_lib, "lib.xml")
            with open(lib_path, "w") as f:
                f.write(
                    """<root>
                  <BehaviorTree ID="LibTree">
                    <Sequence name="MySeq">
                        <Producer name="P" output="{o}"/>
                    </Sequence>
                  </BehaviorTree>
                </root>"""
                )

            main_path = os.path.join(d_main, "main.xml")
            with open(main_path, "w") as f:
                f.write(
                    """<root main_tree_to_execute="Main">
                  <Include src="lib.xml"/>
                  <BehaviorTree ID="Main">
                    <Sequence>
                      <SubTree ID="LibTree" name="L" o="{f}"/>
                      <Consumer name="C" input="{f}"/>
                    </Sequence>
                  </BehaviorTree>
                </root>"""
                )

            root = parse_behaviour_tree_xml(
                main_path,
                logger=StdoutLogger(),
                search_paths=[d_lib],  # key part of this test
            )
            tree = py_trees.trees.BehaviourTree(root)
            tree.tick()
            c = find_node_by_name(root, "C", strip_prefix=True)
            assert isinstance(c, Consumer)
            self.assertEqual(c.consumed_value, "Producer[/L:L.MySeq.P]")

    def test_include_subtree_file_with_search_paths(self) -> None:
        """A subtree <Include>d from a sibling directory resolves via 'search_paths'.

        Uses the on-disk fixtures in tests/xml/: the main tree includes
        'subtree_library.xml', which lives in tests/xml/subtrees/ and is therefore
        only found when that directory is passed via 'search_paths'.
        """
        xml_dir = os.path.join(os.path.dirname(__file__), "xml")
        main_path = os.path.join(xml_dir, "main_tree_with_include.xml")
        root = parse_behaviour_tree_xml(
            main_path,
            logger=StdoutLogger(),
            search_paths=[os.path.join(xml_dir, "subtrees")],
        )
        tree = py_trees.trees.BehaviourTree(root)
        tree.tick()
        consumer = find_node_by_name(root, "FinalConsumer", strip_prefix=True)
        assert isinstance(consumer, Consumer)
        self.assertEqual(consumer.consumed_value, "Producer[/Library:Library.LibSeq.LibProducer]")

    def test_include_subtree_file_without_search_paths_fails(self) -> None:
        """Without 'search_paths', the include in tests/xml/ cannot be resolved."""
        xml_dir = os.path.join(os.path.dirname(__file__), "xml")
        main_path = os.path.join(xml_dir, "main_tree_with_include.xml")
        with self.assertRaises(FileNotFoundError):
            parse_behaviour_tree_xml(main_path, logger=StdoutLogger())

    def test_imported_bt_missing_id(self) -> None:
        """Imported file containing a <BehaviorTree> without an ID raises ValueError."""
        lib_path = self._write_temp_xml("""<root><BehaviorTree><Sequence/></BehaviorTree></root>""")
        main_path = self._write_temp_xml(
            f"""<root main_tree_to_execute="Main">
          <Import src="{lib_path}"/>
          <BehaviorTree ID="Main"><Sequence/></BehaviorTree>
        </root>"""
        )
        try:
            with self.assertRaises(ValueError):
                parse_behaviour_tree_xml(main_path, logger=StdoutLogger())
        finally:
            os.unlink(lib_path)
            os.unlink(main_path)

    def test_parsing_floating_point_direct_values(self) -> None:
        """Verify that floating point direct values are successfully parsed via the XML parser."""
        self.xml = """<root main_tree_to_execute="MainTree">
          <BehaviorTree ID="MainTree">
            <Sequence>
              <Consumer name="cons" input="10.0" />
            </Sequence>
          </BehaviorTree>
          </root>"""

        self.tempfile = tempfile.NamedTemporaryFile(delete=False, mode="w", suffix=".xml")
        self.tempfile.write(self.xml)
        self.tempfile.close()

        root_node = parse_behaviour_tree_xml(self.tempfile.name, logger=StdoutLogger())
        btree = py_trees.trees.BehaviourTree(root_node)

        btree.tick()

        node = find_node_by_name(root_node, "cons", strip_prefix=True)
        assert isinstance(node, Consumer)

        self.assertEqual(node.consumed_value, "10.0")

    def test_type_coercion_with_float_consumer(self) -> None:
        """Test that string values are converted to float when appropriate."""
        from .test_ports_helpers import FloatConsumer

        xml = """<root main_tree_to_execute="MainTree">
        <BehaviorTree ID="MainTree">
          <Sequence>
            <FloatConsumer name="float_cons" input="3.14" />
          </Sequence>
        </BehaviorTree>
        </root>"""

        with tempfile.NamedTemporaryFile(delete=False, mode="w", suffix=".xml") as tf:
            tf.write(xml)
            temp_xml_path = tf.name

        try:
            root_node = parse_behaviour_tree_xml(temp_xml_path, logger=StdoutLogger())
            btree = py_trees.trees.BehaviourTree(root_node)
            btree.tick()

            node = find_node_by_name(root_node, "float_cons", strip_prefix=True)
            assert isinstance(node, FloatConsumer)
            self.assertEqual(node.consumed_value, 3.14)
            self.assertIsInstance(node.consumed_value, float)
        finally:
            os.unlink(temp_xml_path)


class TestNodeRegistry(unittest.TestCase):
    """The ``node_registry`` argument: ``"auto"`` vs an explicit dict."""

    XML = """<root main_tree_to_execute="MainTree">
        <BehaviorTree ID="MainTree">
          <Sequence>
            <Producer name="prod" output="{final}" />
            <Consumer name="cons" input="{final}" />
          </Sequence>
        </BehaviorTree>
      </root>"""

    def setUp(self) -> None:
        py_trees.blackboard.Blackboard.clear()
        # Producer/Consumer are concrete BehaviourWithPorts -> auto-registered on import.
        self.tempfile = tempfile.NamedTemporaryFile(delete=False, mode="w", suffix=".xml")
        self.tempfile.write(self.XML)
        self.tempfile.close()

    def tearDown(self) -> None:
        os.unlink(self.tempfile.name)

    def test_auto_is_the_default(self) -> None:
        """With no node_registry, every tag resolves from the auto-registry."""
        root_node = parse_behaviour_tree_xml(self.tempfile.name)
        py_trees.trees.BehaviourTree(root_node).tick()
        cons = find_node_by_name(root_node, "cons", strip_prefix=True)
        assert isinstance(cons, Consumer)
        self.assertEqual(cons.consumed_value, "Producer[/:prod]")

    def test_auto_string_is_equivalent_to_default(self) -> None:
        """Passing node_registry="auto" explicitly behaves like the default."""
        root_node = parse_behaviour_tree_xml(self.tempfile.name, node_registry="auto")
        cons = find_node_by_name(root_node, "cons", strip_prefix=True)
        self.assertIsInstance(cons, Consumer)

    def test_invalid_registry_string_raises(self) -> None:
        with self.assertRaises(ValueError):
            parse_behaviour_tree_xml(self.tempfile.name, node_registry="nope")

    def test_non_dict_non_str_registry_raises(self) -> None:
        with self.assertRaises(TypeError):
            parse_behaviour_tree_xml(
                self.tempfile.name,
                node_registry=["Producer"],  # type: ignore
            )

    def test_empty_dict_raises(self) -> None:
        """An explicit empty registry leaves the parser with no classes."""
        with self.assertRaises(ValueError):
            parse_behaviour_tree_xml(self.tempfile.name, node_registry={})

    def test_explicit_dict_replaces_auto(self) -> None:
        """A dict is used exclusively: a tag missing from it is not resolved."""
        with self.assertRaises(ValueError):
            parse_behaviour_tree_xml(
                self.tempfile.name,
                node_registry={"Producer": Producer},  # Consumer deliberately omitted
            )

    def test_explicit_entry_overrides_auto_via_spread(self) -> None:
        """Spreading the auto-registry lets one entry override a single tag."""

        class OverrideProducer(Producer, register=False):
            pass

        root_node = parse_behaviour_tree_xml(
            self.tempfile.name,
            node_registry={**get_ports_registry(), "Producer": OverrideProducer},
        )
        prod = find_node_by_name(root_node, "prod", strip_prefix=True)
        self.assertIsInstance(prod, OverrideProducer)

    def test_partial_injection_alongside_auto(self) -> None:
        """The documented pattern: auto-registry spread plus a partial for DI."""
        xml = """<root main_tree_to_execute="MainTree">
            <BehaviorTree ID="MainTree">
              <Sequence>
                <Producer name="prod" output="{final}" />
                <Wait name="w" input_duration_ms="0" />
              </Sequence>
            </BehaviorTree>
          </root>"""
        with tempfile.NamedTemporaryFile(delete=False, mode="w", suffix=".xml") as tf:
            tf.write(xml)
            temp_xml_path = tf.name
        try:
            # Producer resolves via the auto-registry spread; Wait needs a `factory`
            # dependency, supplied via a partial that only the per-call dict can carry.
            root_node = parse_behaviour_tree_xml(
                temp_xml_path,
                node_registry={
                    **get_ports_registry(),
                    "Wait": partial(Wait, factory=DummyFactory()),
                },
            )
            wait = find_node_by_name(root_node, "w", strip_prefix=True)
            prod = find_node_by_name(root_node, "prod", strip_prefix=True)
            self.assertIsInstance(wait, Wait)
            self.assertIsInstance(prod, Producer)
        finally:
            os.unlink(temp_xml_path)


if __name__ == "__main__":
    unittest.main()

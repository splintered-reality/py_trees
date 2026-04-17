# ruff: noqa: SIM115
import os
import tempfile
import time
import unittest
from dataclasses import dataclass
from functools import partial

import py_trees

from py_trees.ports import BehaviourWithPorts
from py_trees._ports_utils import (
    find_node_by_class,
    find_node_by_name,
    generate_node_name,
    strip_trailing_uuid4,
)
from py_trees.parsers.behaviour_tree_xml import parse_behaviour_tree_xml

from .test_ports_helpers import Consumer, ConsumerProducer, Producer


class StdoutLogger:
    """Simple stdout logger for test debug output (replaces the removed standalone StdoutLogger class)."""

    def info(self, msg: str):
        print(f"[INFO] {msg}")

    def warning(self, msg: str):
        print(f"[WARNING] {msg}")

    def error(self, msg: str):
        print(f"[ERROR] {msg}")

    def debug(self, msg: str):
        print(f"[DEBUG] {msg}")


class DummyFactory:
    pass


class Wait(BehaviourWithPorts):
    INPUT_DURATION_MS_PORT = "input_duration_ms"

    def __init__(self, name: str, factory: DummyFactory, **kwargs):
        super().__init__(name=name, **kwargs)
        self._factory = factory
        self.start_time = 0.0

    @classmethod
    def input_ports(cls):
        return {cls.INPUT_DURATION_MS_PORT: (int, True)}

    @classmethod
    def output_ports(cls):
        return {}

    def initialise(self):
        self.start_time = time.time()

    def update(self):
        if self.duration_value_ms < 0:
            return py_trees.common.Status.RUNNING
        return (
            py_trees.common.Status.SUCCESS
            if (time.time() - self.start_time) >= self.duration_value_ms / 1000.0
            else py_trees.common.Status.RUNNING
        )

    @property
    def duration_value_ms(self):
        return self.get_input(self.INPUT_DURATION_MS_PORT)


@dataclass
class RobotData:
    type: str
    ordered_joint_names: list[str]
    commander: object | None


def get_behaviors_lookup(factory: DummyFactory, _robot_data: dict[int, RobotData]) -> dict:
    return {"Wait": partial(Wait, factory=factory)}


class TestXMLParser(unittest.TestCase):
    # Find the final consumer node

    def setUp(self):
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
        # BehaviourWithPorts lookup for test helpers
        self.init_lookup = {
            "Producer": Producer,
            "Consumer": Consumer,
            "ConsumerProducer": ConsumerProducer,
        }

        self.factory = DummyFactory()

    def tearDown(self):
        os.unlink(self.tempfile.name)

    def test_xml_parser_remapping(self):
        """Ensure remapping between subtrees works correctly."""
        root_node = parse_behaviour_tree_xml(self.tempfile.name, init_lookup=self.init_lookup, logger=StdoutLogger())
        # Wrap in a py_trees BehaviourTree and tick until complete
        btree = py_trees.trees.BehaviourTree(root_node)
        btree.tick()
        cons = find_node_by_class(btree.root, Consumer)
        self.assertIsNotNone(cons)
        self.assertEqual(cons.consumed_value, "Producer[/SubTree:SubTree.prod]")

    def test_grandparent_xml(self):
        """Test XML parser with grandparent-child relationships and correct value propagation."""
        xml_path = os.path.join(os.path.dirname(__file__), "grandparent_test.xml")
        root_node = parse_behaviour_tree_xml(xml_path, init_lookup=self.init_lookup, logger=StdoutLogger())
        btree = py_trees.trees.BehaviourTree(root_node)
        btree.tick()
        cons = find_node_by_name(btree.root, generate_node_name("ConsumerMain"))
        self.assertIsNotNone(cons)
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

    def test_custom_behavior_with_extra_arg(self):
        """Test custom behavior with an additional argument."""

        class CustomBehaviourWithPorts(BehaviourWithPorts):
            @classmethod
            def input_ports(cls):
                return {"in": (str, False)}

            @classmethod
            def output_ports(cls):
                return {"out": (str, False)}

            def __init__(self, name, extra_arg, **kwargs):
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

        # Use partial to provide the extra argument
        custom_lookup = dict(self.init_lookup)
        custom_lookup["CustomBehaviourWithPorts"] = partial(CustomBehaviourWithPorts, extra_arg="hello-world")

        try:
            root_node = parse_behaviour_tree_xml(temp_xml_path, init_lookup=custom_lookup, logger=StdoutLogger())
            custom = find_node_by_class(root_node, CustomBehaviourWithPorts)
            self.assertIsNotNone(custom)
            self.assertEqual(custom.extra_arg, "hello-world")
        finally:
            os.unlink(temp_xml_path)

    def test_subtree_remapping_only_explicit_keys(self):
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

        init_lookup = {
            "Consumer": Consumer,
            "Producer": Producer,
        }

        root = parse_behaviour_tree_xml(temp_xml_path, init_lookup=init_lookup)
        btree = py_trees.trees.BehaviourTree(root)
        btree.tick()

        # MyInternalConsumer does not get the {transfer_key} in the main namespace
        node = find_node_by_name(root, "MyInternalConsumer", strip_prefix=True)
        self.assertIsNotNone(node)

        expected_value = "Producer[/Subtree1:Subtree1.MyInternalProducer]"
        self.assertEqual(node.consumed_value, expected_value)

    def test_tree_structure(self):
        """Verify that the structure of the behavior tree matches the XML."""
        root_node = parse_behaviour_tree_xml(self.tempfile.name, init_lookup=self.init_lookup, logger=StdoutLogger())
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

    def test_parsing_simple_direct_values_to_XML(self):
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

        root_node = parse_behaviour_tree_xml(self.tempfile.name, init_lookup=self.init_lookup, logger=StdoutLogger())
        btree = py_trees.trees.BehaviourTree(root_node)

        btree.tick()

        node = find_node_by_name(root_node, "cons")
        self.assertIsNotNone(node)

        self.assertEqual(type(node.consumed_value), str)
        self.assertEqual(node.consumed_value, "ABC")

    def test_parsing_direct_values_to_XML(self):
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

        root_node = parse_behaviour_tree_xml(self.tempfile.name, init_lookup=self.init_lookup, logger=StdoutLogger())
        btree = py_trees.trees.BehaviourTree(root_node)

        btree.tick()

        node = find_node_by_name(root_node, "internalcons", strip_prefix=True)
        self.assertIsNotNone(node)

        expected_value = "Producer[/Subtree1:Subtree1.prod]"
        self.assertEqual(node.consumed_value, expected_value)

        node = find_node_by_name(root_node, "cons", strip_prefix=True)
        self.assertIsNotNone(node)

        expected_value = "500"

        self.assertEqual(node.consumed_value, expected_value)

    def test_subtree_parsing_direct_values_to_XML(self):
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

        root_node = parse_behaviour_tree_xml(self.tempfile.name, init_lookup=self.init_lookup, logger=StdoutLogger())
        btree = py_trees.trees.BehaviourTree(root_node)

        btree.tick()

        node = find_node_by_name(root_node, "InternalConsumer3", strip_prefix=True)
        self.assertIsNotNone(node)

        expected_value = "Producer[/SubTree1:SubTree1.Producer2]"
        self.assertEqual(node.consumed_value, expected_value)

        node = find_node_by_name(root_node, "Consumer1", strip_prefix=True)
        self.assertIsNotNone(node)

        expected_value = "sunrise"

        self.assertEqual(node.consumed_value, expected_value)

        node = find_node_by_name(root_node, "Consumer2", strip_prefix=True)
        self.assertIsNotNone(node)

        expected_value = "100"

        self.assertEqual(node.consumed_value, expected_value)

    def test_wait_node(self):
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

        init_lookup = {"Wait": partial(Wait, factory=self.factory)}

        root_node = parse_behaviour_tree_xml(self.tempfile.name, init_lookup=init_lookup, logger=StdoutLogger())
        btree = py_trees.trees.BehaviourTree(root_node)

        start_time = time.time()
        while btree.root.status != py_trees.common.Status.SUCCESS:
            btree.tick()
            time.sleep(0.01)
        duration = time.time() - start_time

        node = find_node_by_name(root_node, "Pause1", strip_prefix=True)
        self.assertIsNotNone(node)

        self.assertEqual(node.duration_value_ms, wait_duration_ms)
        self.assertAlmostEqual(duration, wait_duration_ms / 1000.0, delta=1)

    def test_wait_with_registry_node(self):
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
        SMALL_JOINT_NAMES = [f'robot_small_joint_{i}' for i in range(1, 7)]
        BIG_JOINT_NAMES = [f'robot_big_joint_{i}' for i in range(1, 7)]
        init_lookup = get_behaviors_lookup(
            self.factory,
            {0: RobotData('robot_small', SMALL_JOINT_NAMES, None), 1: RobotData('robot_big', BIG_JOINT_NAMES, None)},
        )

        root_node = parse_behaviour_tree_xml(self.tempfile.name, init_lookup=init_lookup, logger=StdoutLogger())
        btree = py_trees.trees.BehaviourTree(root_node)

        start_time = time.time()
        while btree.root.status != py_trees.common.Status.SUCCESS:
            btree.tick()
            time.sleep(0.01)
        duration = time.time() - start_time

        node = find_node_by_name(root_node, "Pause1", strip_prefix=True)
        self.assertIsNotNone(node)

        self.assertEqual(node.duration_value_ms, wait_duration_ms)
        self.assertAlmostEqual(duration, wait_duration_ms / 1000.0, delta=1)

    def test_ctor_args_passed_as_kwargs(self):
        """
        Non-port XML attributes must be passed as constructor kwargs.
        Values are left as strings (no automatic type coercion).
        """

        class EchoCtorArgs(BehaviourWithPorts):
            @classmethod
            def input_ports(cls):
                return {"in": (str, False)}  # not used here

            @classmethod
            def output_ports(cls):
                return {"out": (str, False)}  # not used here

            def __init__(self, name, greeting, times, flag, **kwargs):
                super().__init__(name, **kwargs)
                self.greeting = greeting
                self.times = times
                self.flag = flag

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

        try:
            init_lookup = dict(self.init_lookup)
            init_lookup["EchoCtorArgs"] = EchoCtorArgs

            root = parse_behaviour_tree_xml(path, init_lookup=init_lookup, logger=StdoutLogger())
            node = find_node_by_class(root, EchoCtorArgs)
            self.assertIsNotNone(node)
            # No auto type-casting: still strings
            self.assertEqual(node.greeting, "hello")
            self.assertEqual(node.times, "3")
            self.assertEqual(node.flag, "true")
        finally:
            os.unlink(path)

    def test_mixed_ports_and_ctor_kwargs(self):
        """
        If an attribute matches a declared port, it is handled as a port (remap/resolve)
        and must NOT be passed as a constructor kwarg. Non-port attributes become ctor kwargs.
        """

        class PortAndCtor(BehaviourWithPorts):
            @classmethod
            def input_ports(cls):
                return {"in": (str, True)}  # only this is a port

            @classmethod
            def output_ports(cls):
                return {"out": (str, False)}

            def __init__(self, name, label, **kwargs):
                super().__init__(name, **kwargs)
                self.label = label
                self.in_value = None

            def update(self):
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
            init_lookup = dict(self.init_lookup)
            init_lookup["PortAndCtor"] = PortAndCtor

            root = parse_behaviour_tree_xml(path, init_lookup=init_lookup, logger=StdoutLogger())
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

    def test_ctor_arg_with_curly_value_is_not_resolved(self):
        """
        Non-port attributes that look like keys (e.g., "{foo}") raise an exception.
        """

        class TakesKeyString(BehaviourWithPorts):
            @classmethod
            def input_ports(cls):
                return {}  # no ports at all

            @classmethod
            def output_ports(cls):
                return {}

            def __init__(self, name, token):
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

        init_lookup = dict(self.init_lookup)
        init_lookup["TakesKeyString"] = TakesKeyString
        with self.assertRaises(ValueError):
            root = parse_behaviour_tree_xml(path, init_lookup=init_lookup, logger=StdoutLogger())
        os.unlink(path)


class TestXMLParserImports(unittest.TestCase):
    """Tests for XML import pre-processing (top-level <Import>/<Include>)."""

    def setUp(self):
        py_trees.blackboard.Blackboard.clear()
        # Reuse simple helpers
        self.init_lookup = {
            "Producer": Producer,
            "Consumer": Consumer,
            "ConsumerProducer": ConsumerProducer,
        }

    def _write_temp_xml(self, content: str) -> str:
        tf = tempfile.NamedTemporaryFile(delete=False, mode="w", suffix=".xml")
        tf.write(content)
        tf.close()
        return tf.name

    def tearDown(self):
        # Nothing to do; each test cleans its own temps
        pass

    def test_import_basic(self):
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
            root = parse_behaviour_tree_xml(main_path, init_lookup=self.init_lookup, logger=StdoutLogger())
            tree = py_trees.trees.BehaviourTree(root)
            tree.tick()
            c = find_node_by_name(root, "C", strip_prefix=True)
            self.assertIsNotNone(c)
            self.assertEqual(c.consumed_value, "Producer[/Lib1:MYSEQ.Lib1.lib_prod]")
        finally:
            os.unlink(lib_path)
            os.unlink(main_path)

    def test_import_duplicate_id_conflict_with_local(self):
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
                parse_behaviour_tree_xml(main_path, init_lookup=self.init_lookup, logger=StdoutLogger())
        finally:
            os.unlink(lib_path)
            os.unlink(main_path)

    def test_import_duplicate_id_conflict_between_imports(self):
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
                parse_behaviour_tree_xml(main_path, init_lookup=self.init_lookup, logger=StdoutLogger())
        finally:
            os.unlink(path_a)
            os.unlink(path_b)
            os.unlink(main_path)

    def test_import_missing_file(self):
        """Missing import target raises FileNotFoundError."""
        main_xml = """<root main_tree_to_execute="Main">
          <Import src="/does/not/exist/lib.xml"/>
          <BehaviorTree ID="Main"><Sequence/></BehaviorTree>
        </root>"""
        main_path = self._write_temp_xml(main_xml)
        try:
            with self.assertRaises(FileNotFoundError):
                parse_behaviour_tree_xml(main_path, init_lookup=self.init_lookup, logger=StdoutLogger())
        finally:
            os.unlink(main_path)

    def test_import_main_tree_from_import(self):
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
            root = parse_behaviour_tree_xml(main_path, init_lookup=self.init_lookup, logger=StdoutLogger())
            tree = py_trees.trees.BehaviourTree(root)
            tree.tick()
            x = find_node_by_name(root, "X", strip_prefix=True)
            self.assertIsNotNone(x)
            self.assertEqual(x.consumed_value, "OK")
        finally:
            os.unlink(lib_path)
            os.unlink(main_path)

    def test_import_nested_not_supported(self):
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
                parse_behaviour_tree_xml(main_path, init_lookup=self.init_lookup, logger=StdoutLogger())
        finally:
            os.unlink(lib_path)
            os.unlink(main_path)

    def test_import_with_search_paths(self):
        """Import path can be resolved via the 'search_paths' argument."""
        with tempfile.TemporaryDirectory() as d_main, tempfile.TemporaryDirectory() as d_lib:
            lib_path = os.path.join(d_lib, "lib.xml")
            with open(lib_path, "w") as f:
                f.write("""<root>
                  <BehaviorTree ID="LibTree">
                    <Sequence name="MySeq">
                        <Producer name="P" output="{o}"/>
                    </Sequence>
                  </BehaviorTree>
                </root>""")

            main_path = os.path.join(d_main, "main.xml")
            with open(main_path, "w") as f:
                f.write("""<root main_tree_to_execute="Main">
                  <Include src="lib.xml"/>
                  <BehaviorTree ID="Main">
                    <Sequence>
                      <SubTree ID="LibTree" name="L" o="{f}"/>
                      <Consumer name="C" input="{f}"/>
                    </Sequence>
                  </BehaviorTree>
                </root>""")

            root = parse_behaviour_tree_xml(
                main_path,
                init_lookup=self.init_lookup,
                logger=StdoutLogger(),
                search_paths=[d_lib],  # key part of this test
            )
            tree = py_trees.trees.BehaviourTree(root)
            tree.tick()
            c = find_node_by_name(root, "C", strip_prefix=True)
            self.assertIsNotNone(c)
            self.assertEqual(c.consumed_value, "Producer[/L:L.MySeq.P]")

    def test_imported_bt_missing_id(self):
        """Imported file containing a <BehaviorTree> without an ID raises ValueError."""
        lib_path = self._write_temp_xml("""<root><BehaviorTree><Sequence/></BehaviorTree></root>""")
        main_path = self._write_temp_xml(f"""<root main_tree_to_execute="Main">
          <Import src="{lib_path}"/>
          <BehaviorTree ID="Main"><Sequence/></BehaviorTree>
        </root>""")
        try:
            with self.assertRaises(ValueError):
                parse_behaviour_tree_xml(main_path, init_lookup=self.init_lookup, logger=StdoutLogger())
        finally:
            os.unlink(lib_path)
            os.unlink(main_path)

    def test_parsing_floating_point_direct_values(self):
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

        root_node = parse_behaviour_tree_xml(self.tempfile.name, init_lookup=self.init_lookup, logger=StdoutLogger())
        btree = py_trees.trees.BehaviourTree(root_node)

        btree.tick()

        node = find_node_by_name(root_node, "cons", strip_prefix=True)
        self.assertIsNotNone(node)

        self.assertEqual(node.consumed_value, "10.0")

    def test_type_coercion_with_float_consumer(self):
        """Test that string values are converted to float when appropriate."""
        from .test_ports_helpers import FloatConsumer

        xml = """<root main_tree_to_execute="MainTree">
        <BehaviorTree ID="MainTree">
          <Sequence>
            <FloatConsumer name="float_cons" input="3.14" />
          </Sequence>
        </BehaviorTree>
        </root>"""

        init_lookup = dict(self.init_lookup)
        init_lookup["FloatConsumer"] = FloatConsumer

        with tempfile.NamedTemporaryFile(delete=False, mode="w", suffix=".xml") as tf:
            tf.write(xml)
            temp_xml_path = tf.name

        try:
            root_node = parse_behaviour_tree_xml(temp_xml_path, init_lookup=init_lookup, logger=StdoutLogger())
            btree = py_trees.trees.BehaviourTree(root_node)
            btree.tick()

            node = find_node_by_name(root_node, "float_cons", strip_prefix=True)
            self.assertIsNotNone(node)
            self.assertEqual(node.consumed_value, 3.14)
            self.assertIsInstance(node.consumed_value, float)
        finally:
            os.unlink(temp_xml_path)


if __name__ == "__main__":
    unittest.main()

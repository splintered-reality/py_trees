#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/splintered-reality/py_trees/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""Parse and run a behaviour tree from XML with remapping and subtrees."""

##############################################################################
# Imports
##############################################################################

from pathlib import Path

import py_trees
from py_trees.parsers.behaviour_tree_xml import parse_behaviour_tree_xml
from py_trees.ports import BehaviourWithPorts, PortInformation
from py_trees.ports_utils import find_node_by_class

##############################################################################
# Classes
##############################################################################


class GreetingProducer(BehaviourWithPorts):
    """Produce a greeting string."""

    @classmethod
    def input_ports(cls) -> dict:
        """Return the input port declarations."""
        return {}

    @classmethod
    def output_ports(cls) -> dict:
        """Return the output port declarations."""
        return {"output": PortInformation(data_type=str, required=True)}

    def update(self) -> py_trees.common.Status:
        """Write a fixed greeting to the output port."""
        self._set_output("output", "hello")
        return py_trees.common.Status.SUCCESS


class AddSuffix(BehaviourWithPorts):
    """Append a configurable suffix to an input string."""

    @classmethod
    def input_ports(cls) -> dict:
        """Return the input port declarations."""
        return {
            "input": PortInformation(data_type=str, required=True),
            "suffix": PortInformation(data_type=str, required=True),
        }

    @classmethod
    def output_ports(cls) -> dict:
        """Return the output port declarations."""
        return {"output": PortInformation(data_type=str, required=True)}

    def update(self) -> py_trees.common.Status:
        """Append ``suffix`` to ``input`` and write the result to ``output``."""
        self._set_output("output", f"{self.get_input('input')}{self.get_input('suffix')}")
        return py_trees.common.Status.SUCCESS


class PrintConsumer(BehaviourWithPorts):
    """Capture the final value for display."""

    @classmethod
    def input_ports(cls) -> dict:
        """Return the input port declarations."""
        return {"input": PortInformation(data_type=str, required=True)}

    @classmethod
    def output_ports(cls) -> dict:
        """Return the output port declarations."""
        return {}

    def update(self) -> py_trees.common.Status:
        """Return SUCCESS; the value is read via :attr:`consumed_value`."""
        return py_trees.common.Status.SUCCESS

    @property
    def consumed_value(self) -> str:
        """Return the value wired to this node's input port."""
        value: str = self.get_input("input")
        return value


##############################################################################
# Main
##############################################################################


def main() -> None:
    """Parse XML, tick the tree once, and print the consumed final message."""
    py_trees.blackboard.Blackboard.clear()

    xml_file = Path(__file__).with_name("xml_tree.xml")
    root = parse_behaviour_tree_xml(str(xml_file))

    tree = py_trees.trees.BehaviourTree(root)
    tree.tick()

    consumer = find_node_by_class(tree.root, PrintConsumer)
    print("final_message =", consumer.consumed_value)


if __name__ == "__main__":
    main()

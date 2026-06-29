#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/splintered-reality/py_trees/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""Nested subtrees for a robotics pickup mission."""

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


class StartMissionReport(BehaviourWithPorts):
    """Create the initial mission report text."""

    @classmethod
    def input_ports(cls) -> dict:
        """Return the input port declarations."""
        return {
            "robot_name": PortInformation(data_type=str, required=True),
            "mission_name": PortInformation(data_type=str, required=True),
        }

    @classmethod
    def output_ports(cls) -> dict:
        """Return the output port declarations."""
        return {"report": PortInformation(data_type=str, required=True)}

    def update(self) -> py_trees.common.Status:
        """Combine ``robot_name`` and ``mission_name`` into the initial report."""
        robot_name = self.get_input("robot_name")
        mission_name = self.get_input("mission_name")
        self._set_output("report", f"{robot_name}: {mission_name}")
        return py_trees.common.Status.SUCCESS


class AddMissionStep(BehaviourWithPorts):
    """Append one mission step to the report text."""

    @classmethod
    def input_ports(cls) -> dict:
        """Return the input port declarations."""
        return {
            "report_in": PortInformation(data_type=str, required=True),
            "step": PortInformation(data_type=str, required=True),
        }

    @classmethod
    def output_ports(cls) -> dict:
        """Return the output port declarations."""
        return {"report_out": PortInformation(data_type=str, required=True)}

    def update(self) -> py_trees.common.Status:
        """Append ``step`` to ``report_in`` and write to ``report_out``."""
        current_report = self.get_input("report_in")
        step_name = self.get_input("step")
        self._set_output("report_out", f"{current_report} -> {step_name}")
        return py_trees.common.Status.SUCCESS


class ReadMissionReport(BehaviourWithPorts):
    """Final node that exposes the finished report for printing."""

    @classmethod
    def input_ports(cls) -> dict:
        """Return the input port declarations."""
        return {"report": PortInformation(data_type=str, required=True)}

    @classmethod
    def output_ports(cls) -> dict:
        """Return the output port declarations."""
        return {}

    def update(self) -> py_trees.common.Status:
        """Return SUCCESS; the report is read via :attr:`final_report`."""
        return py_trees.common.Status.SUCCESS

    @property
    def final_report(self) -> str:
        """Return the final report wired to this node's input port."""
        report: str = self.get_input("report")
        return report


##############################################################################
# Main
##############################################################################


def main() -> None:
    """Parse nested subtrees from XML and execute one mission tick."""
    py_trees.blackboard.Blackboard.clear()

    xml_file = Path(__file__).with_name("nested_subtrees.xml")
    root = parse_behaviour_tree_xml(str(xml_file))
    tree = py_trees.trees.BehaviourTree(root)
    tree.tick()

    report_reader = find_node_by_class(tree.root, ReadMissionReport)
    print("Final mission report:")
    print(report_reader.final_report)


if __name__ == "__main__":
    main()

#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/splintered-reality/py_trees/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""Minimal typed input/output ports with a single behaviour node."""

##############################################################################
# Imports
##############################################################################

import py_trees
from py_trees.ports import BehaviourWithPorts, PortInformation

##############################################################################
# Classes
##############################################################################


class Multiply(BehaviourWithPorts):
    """Read two floats and write their product."""

    @classmethod
    def input_ports(cls) -> dict:
        """Return the input port declarations."""
        return {
            "a": PortInformation(data_type=float, required=True),
            "b": PortInformation(data_type=float, required=True),
        }

    @classmethod
    def output_ports(cls) -> dict:
        """Return the output port declarations."""
        return {
            "product": PortInformation(data_type=float, required=True),
        }

    def update(self) -> py_trees.common.Status:
        """Multiply the two inputs and write the result to the output port."""
        self._set_output("product", self.get_input("a") * self.get_input("b"))
        return py_trees.common.Status.SUCCESS


##############################################################################
# Helpers
##############################################################################


def seed_value(key: str, value: float) -> None:
    """Write one value to an absolute blackboard key."""
    seeder = py_trees.blackboard.Client(name="seeder")
    seeder.register_key(key=key, access=py_trees.common.Access.WRITE, required=True)
    seeder.set(key, value)


##############################################################################
# Main
##############################################################################


def main() -> None:
    """Run a single-node demo with explicit key remapping."""
    py_trees.blackboard.Blackboard.clear()

    node = Multiply(name="mul")
    node.setup_ports(
        port_remappings={
            "a": "/numbers/a",
            "b": "/numbers/b",
            "product": "/numbers/product",
        },
        subtree_namespace="/",
    )

    seed_value("/numbers/a", 2.5)
    seed_value("/numbers/b", 4.0)

    node.tick_once()
    print("a =", node.get_input("a"))
    print("b =", node.get_input("b"))
    print("product =", node.get_last_output("product"))


if __name__ == "__main__":
    main()

#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/splintered-reality/py_trees/devel/LICENSE
#

##############################################################################
# Imports
##############################################################################

from typing import Any

import py_trees
from py_trees.ports import BehaviourWithPorts, PortInformation, PortsMixin


class Producer(BehaviourWithPorts):
    OUTPUT_PORT = "output"

    INPUT_PORTS = {}
    OUTPUT_PORTS = {OUTPUT_PORT: PortInformation(data_type=str, required=True)}

    def update(self) -> py_trees.common.Status:
        self._set_output(self.OUTPUT_PORT, f"Producer[{self.subtree_namespace}:{self.name}]")
        return py_trees.common.Status.SUCCESS


class ConsumerProducer(BehaviourWithPorts):
    OUTPUT_PORT = "output"
    INPUT_PORT = "input"

    INPUT_PORTS = {INPUT_PORT: PortInformation(data_type=str, required=True)}
    OUTPUT_PORTS = {OUTPUT_PORT: PortInformation(data_type=str, required=True)}

    def update(self) -> py_trees.common.Status:
        input_value = self.get_input(self.INPUT_PORT)
        self._set_output(self.OUTPUT_PORT, f"{input_value}[{self.subtree_namespace}:{self.name}]")
        return py_trees.common.Status.SUCCESS


class Consumer(BehaviourWithPorts):
    INPUT_PORT = "input"

    INPUT_PORTS = {INPUT_PORT: PortInformation(data_type=str, required=True)}
    OUTPUT_PORTS = {}

    def update(self) -> py_trees.common.Status:
        return py_trees.common.Status.SUCCESS

    @property
    def consumed_value(self) -> Any:
        return self.get_input(self.INPUT_PORT)


class FloatConsumer(BehaviourWithPorts):
    INPUT_PORT = "input"

    INPUT_PORTS = {INPUT_PORT: PortInformation(data_type=float, required=True)}
    OUTPUT_PORTS = {}

    def update(self) -> py_trees.common.Status:
        return py_trees.common.Status.SUCCESS

    @property
    def consumed_value(self) -> Any:
        return self.get_input(self.INPUT_PORT)


class DefaultingConsumer(BehaviourWithPorts):
    """Consumer whose input ports declare default values."""

    INPUT_PORTS = {
        "input": PortInformation(data_type=str, required=True, default_value="fallback"),
        "items": PortInformation(data_type=list[int], required=False, default_value=[1, 2]),
    }
    OUTPUT_PORTS = {}

    def update(self) -> py_trees.common.Status:
        return py_trees.common.Status.SUCCESS

    @property
    def consumed_value(self) -> Any:
        return self.get_input("input")


class DefaultingProducer(BehaviourWithPorts):
    """Producer whose output ports declare default values."""

    INPUT_PORTS = {}
    OUTPUT_PORTS = {
        "output": PortInformation(data_type=str, default_value="initial"),
        "items": PortInformation(data_type=list[int], default_value=[1, 2]),
    }

    def update(self) -> py_trees.common.Status:
        self._set_output("output", "produced")
        return py_trees.common.Status.SUCCESS


# ---------- Tiny direct-only leaves (no ports needed) ----------


class AlwaysSuccess(py_trees.behaviour.Behaviour):
    def __init__(self, name: str = "S") -> None:
        super().__init__(name)

    def update(self) -> py_trees.common.Status:
        return py_trees.common.Status.SUCCESS


class AlwaysFailure(py_trees.behaviour.Behaviour):
    def __init__(self, name: str = "F") -> None:
        super().__init__(name)

    def update(self) -> py_trees.common.Status:
        return py_trees.common.Status.FAILURE


class AlwaysRunning(py_trees.behaviour.Behaviour):
    def __init__(self, name: str = "R") -> None:
        super().__init__(name)

    def update(self) -> py_trees.common.Status:
        return py_trees.common.Status.RUNNING


class RunsThenSucceeds(py_trees.behaviour.Behaviour):
    """RUNNING on first tick, SUCCESS thereafter."""

    def __init__(self, name: str = "RTS") -> None:
        super().__init__(name)
        self._done = False

    def initialise(self) -> None:
        # nothing special; keep flag as-is
        pass

    def update(self) -> py_trees.common.Status:
        if not self._done:
            self._done = True
            return py_trees.common.Status.RUNNING
        return py_trees.common.Status.SUCCESS


# ---------- XML-friendly leaves (must derive from BehaviourWithPorts) ----------


class AlwaysSuccessBP(BehaviourWithPorts):
    INPUT_PORTS = {}
    OUTPUT_PORTS = {}

    def __init__(self, name: str, **kwargs: Any) -> None:
        super().__init__(name=name, **kwargs)

    def update(self) -> py_trees.common.Status:
        return py_trees.common.Status.SUCCESS


class AlwaysFailureBP(BehaviourWithPorts):
    INPUT_PORTS = {}
    OUTPUT_PORTS = {}

    def __init__(self, name: str, **kwargs: Any) -> None:
        super().__init__(name=name, **kwargs)

    def update(self) -> py_trees.common.Status:
        return py_trees.common.Status.FAILURE


class AlwaysRunningBP(BehaviourWithPorts):
    INPUT_PORTS = {}
    OUTPUT_PORTS = {}

    def __init__(self, name: str, **kwargs: Any) -> None:
        super().__init__(name=name, **kwargs)

    def update(self) -> py_trees.common.Status:
        return py_trees.common.Status.RUNNING


# ---------- Blackboard seeding helpers ----------


def seed_port_value(
    node: PortsMixin,
    port: str,
    value: Any,
    *,
    client_name: str = "Seeder",
    required: bool = True,
) -> str:
    """Write a value to the blackboard location backing ``node``'s ``port``."""
    storage_key = node._get_blackboard_key(port)
    seed_blackboard_value(
        storage_key,
        value,
        namespace=node.subtree_namespace,
        client_name=client_name,
        required=required,
    )
    return storage_key


def seed_port_values(
    node: PortsMixin,
    *,
    client_name: str = "Seeder",
    required: bool = True,
    **port_values: Any,
) -> None:
    """Convenience wrapper to seed multiple ports on a PortsMixin node."""
    for port, value in port_values.items():
        seed_port_value(node, port, value, client_name=client_name, required=required)


def seed_blackboard_value(
    key: str,
    value: Any,
    *,
    namespace: str = "/",
    client_name: str = "Seeder",
    required: bool = True,
) -> str:
    """Seed an absolute blackboard key (bypassing PortsMixin conveniences)."""
    client = py_trees.blackboard.Client(name=client_name, namespace=namespace)
    client.register_key(key=key, access=py_trees.common.Access.WRITE, required=required)
    client.set(key, value)
    return key

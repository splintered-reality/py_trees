from typing import Any

import py_trees

from py_trees.ports import BehaviourWithPorts, PortsMixin


class Producer(BehaviourWithPorts):
    OUTPUT_PORT = "output"

    @classmethod
    def input_ports(cls):
        return {}

    @classmethod
    def output_ports(cls):
        return {cls.OUTPUT_PORT: (str, True)}

    def update(self):
        self._set_output(self.OUTPUT_PORT, f"Producer[{self.subtree_namespace}:{self.name}]")
        return py_trees.common.Status.SUCCESS


class ConsumerProducer(BehaviourWithPorts):
    OUTPUT_PORT = "output"
    INPUT_PORT = "input"

    @classmethod
    def input_ports(cls):
        return {cls.INPUT_PORT: (str, True)}

    @classmethod
    def output_ports(cls):
        return {cls.OUTPUT_PORT: (str, True)}

    def update(self):
        input_value = self.get_input(self.INPUT_PORT)
        self._set_output(self.OUTPUT_PORT, f"{input_value}[{self.subtree_namespace}:{self.name}]")
        return py_trees.common.Status.SUCCESS


class Consumer(BehaviourWithPorts):
    INPUT_PORT = "input"

    @classmethod
    def input_ports(cls):
        return {cls.INPUT_PORT: (str, True)}

    @classmethod
    def output_ports(cls):
        return {}

    def update(self):
        return py_trees.common.Status.SUCCESS

    @property
    def consumed_value(self):
        return self.get_input(self.INPUT_PORT)


class FloatConsumer(BehaviourWithPorts):
    INPUT_PORT = "input"

    @classmethod
    def input_ports(cls):
        return {cls.INPUT_PORT: (float, True)}

    @classmethod
    def output_ports(cls):
        return {}

    def update(self):
        return py_trees.common.Status.SUCCESS

    @property
    def consumed_value(self):
        return self.get_input(self.INPUT_PORT)


# ---------- Tiny direct-only leaves (no ports needed) ----------


class AlwaysSuccess(py_trees.behaviour.Behaviour):
    def __init__(self, name="S"):
        super().__init__(name)

    def update(self):
        return py_trees.common.Status.SUCCESS


class AlwaysFailure(py_trees.behaviour.Behaviour):
    def __init__(self, name="F"):
        super().__init__(name)

    def update(self):
        return py_trees.common.Status.FAILURE


class AlwaysRunning(py_trees.behaviour.Behaviour):
    def __init__(self, name="R"):
        super().__init__(name)

    def update(self):
        return py_trees.common.Status.RUNNING


class RunsThenSucceeds(py_trees.behaviour.Behaviour):
    """RUNNING on first tick, SUCCESS thereafter."""

    def __init__(self, name="RTS"):
        super().__init__(name)
        self._done = False

    def initialise(self):
        # nothing special; keep flag as-is
        pass

    def update(self):
        if not self._done:
            self._done = True
            return py_trees.common.Status.RUNNING
        return py_trees.common.Status.SUCCESS


# ---------- XML-friendly leaves (must derive from BehaviourWithPorts) ----------


class AlwaysSuccessBP(BehaviourWithPorts):
    @classmethod
    def input_ports(cls) -> dict:
        return {}

    @classmethod
    def output_ports(cls) -> dict:
        return {}

    def __init__(self, name: str, **kwargs):
        super().__init__(name=name, **kwargs)

    def update(self) -> py_trees.common.Status:
        return py_trees.common.Status.SUCCESS


class AlwaysFailureBP(BehaviourWithPorts):
    @classmethod
    def input_ports(cls) -> dict:
        return {}

    @classmethod
    def output_ports(cls) -> dict:
        return {}

    def __init__(self, name: str, **kwargs):
        super().__init__(name=name, **kwargs)

    def update(self) -> py_trees.common.Status:
        return py_trees.common.Status.FAILURE


class AlwaysRunningBP(BehaviourWithPorts):
    @classmethod
    def input_ports(cls) -> dict:
        return {}

    @classmethod
    def output_ports(cls) -> dict:
        return {}

    def __init__(self, name: str, **kwargs):
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
):
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
):
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
):
    """Seed an absolute blackboard key (bypassing PortsMixin conveniences)."""
    client = py_trees.blackboard.Client(name=client_name, namespace=namespace)
    client.register_key(key=key, access=py_trees.common.Access.WRITE, required=required)
    client.set(key, value)
    return key

#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/splintered-reality/py_trees/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""Explicit 3-step pipeline with remapping and subtree namespaces."""

##############################################################################
# Imports
##############################################################################

import random
import string
from typing import Any

import py_trees
from py_trees.ports import BehaviourWithPorts, PortInformation

##############################################################################
# Classes
##############################################################################


class GenerateValue(BehaviourWithPorts):
    """Step 1: generate a random value and write it to an output port."""

    def __init__(self, name: str, prefix: str = "value", **kwargs: Any) -> None:
        """Initialise the behaviour with *prefix* used in the generated value."""
        super().__init__(name=name, **kwargs)
        self._prefix = prefix

    @classmethod
    def input_ports(cls) -> dict:
        """Return the input port declarations."""
        return {}

    @classmethod
    def output_ports(cls) -> dict:
        """Return the output port declarations."""
        return {"value": PortInformation(data_type=str, required=True)}

    def update(self) -> py_trees.common.Status:
        """Generate a random value and write it to the output port."""
        token = "".join(random.choices(string.ascii_lowercase + string.digits, k=6))
        self._set_output("value", f"{self._prefix}_{token}")
        return py_trees.common.Status.SUCCESS


class AppendSuffix(BehaviourWithPorts):
    """Step 2: read one string, append a suffix, and write the updated string."""

    def __init__(self, name: str, suffix: str, **kwargs: Any) -> None:
        """Initialise the behaviour with a *suffix* to append."""
        super().__init__(name=name, **kwargs)
        self._suffix = suffix

    @classmethod
    def input_ports(cls) -> dict:
        """Return the input port declarations."""
        return {"text_in": PortInformation(data_type=str, required=True)}

    @classmethod
    def output_ports(cls) -> dict:
        """Return the output port declarations."""
        return {
            "text_out": PortInformation(data_type=str, required=True),
            "value": PortInformation(data_type=str, required=True),
        }

    def update(self) -> py_trees.common.Status:
        """Append the suffix and write both the processed text and a status string."""
        processed_text = f"{self.get_input('text_in')}{self._suffix}"
        self._set_output("text_out", processed_text)
        self._set_output("value", f"@@@ status: written '{processed_text}' @@@")
        return py_trees.common.Status.SUCCESS


class ReadResult(BehaviourWithPorts):
    """Step 3: read the final pipeline value so it can be inspected."""

    @classmethod
    def input_ports(cls) -> dict:
        """Return the input port declarations."""
        return {"value": PortInformation(data_type=str, required=True)}

    @classmethod
    def output_ports(cls) -> dict:
        """Return the output port declarations."""
        return {}

    def update(self) -> py_trees.common.Status:
        """Return SUCCESS; the value is read via :attr:`result`."""
        return py_trees.common.Status.SUCCESS

    @property
    def result(self) -> str:
        """Return the value wired to this node's input port."""
        value: str = self.get_input("value")
        return value


##############################################################################
# Helpers
##############################################################################


def run_pipeline(namespace: str, prefix: str, suffix: str) -> str:
    """
    Run one explicit 3-step pipeline in the given namespace.

    Step 1: GenerateValue writes to ``/step1_value``.

    Step 2: AppendSuffix reads ``/step1_value``, writes ``/step2_value``,
    and also writes a local (unremapped) ``value`` status output.

    Step 3: ReadResult reads ``/step2_value``.
    """
    step1 = GenerateValue(name="step1_generate", prefix=prefix)
    step2 = AppendSuffix(name="step2_append_suffix", suffix=suffix)
    step3 = ReadResult(name="step3_read_result")

    step1.setup_ports(
        port_remappings={"value": "/step1_value"},
        subtree_namespace=namespace,
    )
    step2.setup_ports(
        port_remappings={"text_in": "/step1_value", "text_out": "/step2_value"},
        subtree_namespace=namespace,
    )
    step3.setup_ports(
        port_remappings={"value": "/step2_value"},
        subtree_namespace=namespace,
    )

    step1.tick_once()
    step2.tick_once()
    step3.tick_once()

    print(f"[TEST PROBE] value of unremapped value of step2: {step2.get_last_output('value')}")
    return step3.result


##############################################################################
# Main
##############################################################################


def main() -> None:
    """Run two isolated 3-step pipelines that reuse the same port names (no clash)."""
    py_trees.blackboard.Blackboard.clear()

    pipeline_a = run_pipeline(namespace="/pipeline_a", prefix="alpha", suffix="_done")
    pipeline_b = run_pipeline(namespace="/pipeline_b", prefix="beta", suffix="_done")

    print("==============================")
    print("pipeline_a result:", pipeline_a)
    print("pipeline_b result:", pipeline_b)


if __name__ == "__main__":
    main()

#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/splintered-reality/py_trees/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""Typed input/output ports for py_trees behaviours."""

##############################################################################
# Imports
##############################################################################

import types
import typing
import warnings
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any

import py_trees

from .ports_utils import (
    LogLevel,
    PortsLogger,
    convert_str_to_type,
    reset_blackboard_key,
    sanitize_name_for_blackboard_use,
    set_feedback_and_log,
    strip_trailing_uuid4,
)

# At type-check time, PortsMixin inherits from Behaviour so mypy knows about
# self.name / self.id / self.logger / self.attach_blackboard_client / self.blackboards.
# At runtime, PortsMixin inherits only from ABC; concrete classes combine it with
# Behaviour explicitly (e.g. ``class MyNode(PortsMixin, py_trees.behaviour.Behaviour)``).
if TYPE_CHECKING:
    _MixinBase = py_trees.behaviour.Behaviour
else:
    _MixinBase = ABC

# Const Prefix for parsing direct values from a tree parser
# Any entry having this prefix should be interpreted as a direct value (not key) by a parser
CONST_PREFIX = "__const_"

# Dots are a problem in strings used as direct values in input ports, because a dot is
# also a separator of pytrees namespace. Therefore, we replace it. Only limitation is that
# any values that contain the actual DOT_REPLACEMENT string cannot be used as direct values.
DOT_REPLACEMENT = "__DOT__"


class NoDataAvailable(Exception):  # noqa: N818
    """Exception raised when a required data has not (yet) been written to the port."""

    pass


@dataclass(frozen=True)
class PortInformation:
    """Static declaration for one typed input or output port."""

    data_type: Any
    required: bool = True
    description: str = ""


##############################################################################
# Class registry for tree parsers
##############################################################################

# Maps a tag (lookup name) to a concrete PortsMixin subclass. Populated
# automatically by PortsMixin.__init_subclass__. Defining and importing a
# PortsMixin subclass is enough to make it resolvable by tag, e.g. from a parser.
_REGISTRY: dict[str, type["PortsMixin"]] = {}


def register_ports_class(tag: str, cls: type) -> None:
    """
    Register *cls* under *tag* in the global ports registry.

    Use this to alias a class under an additional tag, or to register a
    third-party :class:`PortsMixin` subclass that cannot be modified to pass
    ``tag=`` / ``register=`` at definition time.

    Args:
        tag: The tag (lookup name) to register under.
        cls: A concrete subclass of :class:`PortsMixin`.

    Raises:
        TypeError: If *cls* is not a subclass of :class:`PortsMixin`.
    """
    if not (isinstance(cls, type) and issubclass(cls, PortsMixin)):
        raise TypeError(f"Cannot register '{tag}': {cls!r} is not a PortsMixin subclass.")
    existing = _REGISTRY.get(tag)
    if existing is not None and existing is not cls:
        warnings.warn(
            f"Ports class tag '{tag}' is already registered to "
            f"'{existing.__name__}'; overriding with '{cls.__name__}' (last wins).",
            stacklevel=2,
        )
    _REGISTRY[tag] = cls


def get_ports_registry() -> dict[str, type["PortsMixin"]]:
    """Return a shallow copy of the global ports registry (``{tag: class}``)."""
    return dict(_REGISTRY)


def _ports_class_is_abstract(cls: type) -> bool:
    """
    Return whether *cls* still has unimplemented abstract methods.

    This intentionally scans the resolved attributes rather than reading
    ``cls.__abstractmethods__``: ``__init_subclass__`` runs *before* ``ABCMeta``
    populates ``__abstractmethods__`` on the new class, so that attribute is not
    yet reliable at registration time.
    """
    return any(getattr(getattr(cls, name, None), "__isabstractmethod__", False) for name in dir(cls))


class PortsMixin(_MixinBase):
    """
    Mixin class for enabling input and output ports on behaviour tree nodes.

    This mixin provides the core infrastructure needed to wire, validate, and execute data-driven
    behaviour tree nodes in a modular and reusable way. It is designed to be used as a base class
    for behaviours, composites, and decorators in behaviour trees.

    A class using ``PortsMixin`` must:

    1. Inherit from ``PortsMixin`` first, followed by a concrete py_trees class
       (e.g. ``py_trees.behaviour.Behaviour``).
    2. Define its input and output ports as class-level information by implementing the
       ``@classmethod`` s ``input_ports`` and ``output_ports``.

    A ``PortsMixin`` represents a modular unit that interacts with input and output data through
    well-defined ports. These ports are typed and validated at runtime to ensure consistency and facilitate
    composability between different nodes.

    Subclasses must define their input and output ports as class-level information by implementing
    the ``@classmethod`` s ``input_ports`` and ``output_ports``.

    * ``input_ports(cls)``: returns a dictionary mapping input port names to port information.
    * ``output_ports(cls)``: returns a dictionary mapping output port names to port information.

    These methods return the expected port definitions for the class and do not change at runtime.
    These port definitions are used to:

    1. Register blackboard keys for communication.
    2. Enforce type and presence validation at runtime.
    3. Provide clear contracts for each behaviour's data dependencies and outputs.

    Example usage::

        class MyBehaviour(PortsMixin, py_trees.behaviour.Behaviour):
            @classmethod
            def input_ports(cls):
                return {"input": PortInformation(data_type=str, required=True)}

            @classmethod
            def output_ports(cls):
                return {"output": PortInformation(data_type=str, required=True)}

            def __init__(self, name: str):
                super().__init__(name=name)

            def update(self):
                input_val = self.get_input("input")
                self._set_output("output", f"Processed({input_val})")
                return py_trees.common.Status.SUCCESS

    Port specification format in ``input_ports()`` and ``output_ports()``::

        {
            "<port_name>": PortInformation(data_type=<expected_type>, required=<bool>),
        }

    Input and output port names must be unique across both sets; overlapping names are not allowed and
    will raise a ``ValueError`` at instantiation.

    **Subtrees**

    ``PortsMixin`` is designed to be used in complex behaviour trees that may consist of multiple subtrees.
    Each behaviour operates within a specified ``subtree_namespace``, allowing multiple instances of the
    same behaviour to run in parallel without interfering with each other's blackboard keys. The namespace
    ensures logical separation between behaviours and enables modular composition of behaviour trees.

    **Blackboard access**

    ``PortsMixin`` operates within a scoped subtree namespace to ensure its blackboard keys don't clash when
    used in multiple subtrees. Always use the ``blackboard`` property, as it provides access to the correctly
    namespaced client. The class abstracts away direct blackboard access in favour of ``get_input()`` and
    ``_set_output()`` methods. Direct access to the blackboard is discouraged and only permitted through the
    ``blackboard`` property after ``setup_ports()`` has been called. However, be aware that the blackboard
    is shared with all other behaviours in the same subtree, so care must be taken to avoid key collisions.

    **Blackboard namespace strategy**

    When a port is **not** explicitly remapped (via a remapping table or constructor arguments), a dedicated storage key
    is generated, so that sibling nodes with the same port name do not accidentally share data. This
    "synthesised" key is derived from:

    - the current subtree namespace,
    - the node's name (sanitised to remove characters that py_trees treats as separators),
    - and the node's UUID.

    Behaviours can continue to share ports by wiring the same absolute key on purpose
    (e.g. ``/shared/output``).

    **Port remapping**

    During setup, ports may be remapped to alternate blackboard keys using the ``port_remappings`` argument.

    **Port setup lifecycle**

    ``setup_ports()`` is a separate explicit call rather than part of ``Behaviour.setup()`` because port
    setup requires the full remapping table, which may require parsing the entire tree to compute.
    The remapping is the "wiring" of ports --- connecting one node's inputs to another node's outputs ---
    so it presupposes knowledge of the tree topology.

    **Output write semantics**

    Output ports are written internally by the node itself (typically inside ``update()``).
    External callers should not write to output ports in production code; writing from outside
    is only expected in unit tests where the blackboard is seeded manually.

    **Composites / decorators scope**

    ``PortsMixin`` can be mixed into any ``Behaviour`` subclass --- leaves, composites, and decorators.
    However, this migration does not ship ports-enabled composite or decorator implementations.
    Those can be contributed in separate follow-up PRs.

    **Example**::

        class ConsumerProducer(PortsMixin, py_trees.behaviour.Behaviour):
            @classmethod
            def input_ports(cls):
                return {"input": PortInformation(data_type=str, required=True)}

            @classmethod
            def output_ports(cls):
                return {"output": PortInformation(data_type=str, required=True)}

            def update(self):
                input_val = self.get_input("input")
                self._set_output("output", f"Processed({input_val})")
                return py_trees.common.Status.SUCCESS
    """

    def __init_subclass__(cls, *, tag: str | None = None, register: bool = True, **kwargs: Any) -> None:
        """
        Auto-register concrete subclasses so parsers can resolve them by tag.

        Defining a concrete ``PortsMixin`` subclass registers it under its class
        name (or *tag*, if given) in the global ports registry, so a parser can
        resolve it by tag.

        Args:
            tag: Optional explicit tag (lookup name). Defaults to ``cls.__name__``.
            register: Set to ``False`` to skip auto-registration for this class.

        Still-abstract subclasses (e.g. :class:`BehaviourWithPorts`, which does
        not implement ``input_ports`` / ``output_ports``) are never registered.
        """
        super().__init_subclass__(**kwargs)
        if not register or _ports_class_is_abstract(cls):
            return
        register_ports_class(tag if tag is not None else cls.__name__, cls)

    @classmethod
    @abstractmethod
    def input_ports(cls) -> dict[str, PortInformation]:
        """Return a mapping of input port names to port information."""
        raise NotImplementedError("Subclasses must implement input_ports()")

    @classmethod
    @abstractmethod
    def output_ports(cls) -> dict[str, PortInformation]:
        """Return a mapping of output port names to port information."""
        raise NotImplementedError("Subclasses must implement output_ports()")

    @classmethod
    def get_port_type(cls, port_name: str) -> type:
        """Return the declared type for *port_name*.

        Args:
            port_name (str): The name of the input or output port.

        Return:
            type: The expected data type of the specified port.

        Raises:
            KeyError: If the port name is not defined in either input or output ports.
        """
        if port_name in cls.input_ports():
            return cls.input_ports()[port_name].data_type  # type: ignore[no-any-return]
        elif port_name in cls.output_ports():
            return cls.output_ports()[port_name].data_type  # type: ignore[no-any-return]
        else:
            raise KeyError(f"Port '{port_name}' not defined.")

    @classmethod
    def is_port_required(cls, port_name: str) -> bool:
        """Return whether the specified port is marked as required.

        Args:
            port_name (str): The name of the input or output port.

        Return:
            bool: True if the specified port is marked as required, False otherwise.

        Raises:
            KeyError: If the port name is not defined in either input or output ports.
        """
        if port_name in cls.input_ports():
            return cls.input_ports()[port_name].required
        elif port_name in cls.output_ports():
            return cls.output_ports()[port_name].required
        else:
            raise KeyError(f"Port '{port_name}' not defined.")

    def __init__(self, *args: Any, behaviour_class_name: str | None = None, **kwargs: Any) -> None:
        """
        Initialize the PortsMixin.

        Args:
            *args: Positional arguments passed to the parent class (typically py_trees.behaviour.Behaviour).
            behaviour_class_name: The name under which this behavior class is registered (e.g., in a registry).
                Typically corresponds to the class name itself, but can also be an alias
                for partial instantiations or custom registrations (with a tree parser, this would be the tag name).
                If None, defaults to the actual class name (self.__class__.__name__).
            **kwargs: Additional keyword arguments passed to the parent class.

        Raises:
            ValueError: If any port name appears in both input_ports() and output_ports().
        """
        super().__init__(*args, **kwargs)
        # The following fields will be added in the setup_ports function and are non-functional intentionally till it
        # gets added to the setup_ports function.
        self._blackboard_client: py_trees.blackboard.Client | None = None
        self._subtree_namespace: str | None = None
        self._ports_logger: PortsLogger | None = None
        # The name under which this behavior class is registered (e.g., in a registry).
        # Usually corresponds to the class name, but can also be an alias for a partial instantiation.
        # Defaults to the actual class name if not provided.
        self._behaviour_class_name = (
            behaviour_class_name if behaviour_class_name is not None else self.__class__.__name__
        )

        # Consistency check: no value can appear in both input and output ports
        for port in self.input_ports():
            if port in self.output_ports():
                raise ValueError(f"Port '{port}' appears in both input and output ports")

    def setup_ports(
        self,
        port_remappings: dict | None = None,
        subtree_namespace: str = "/",
        logger: PortsLogger | None = None,
    ) -> None:
        """
        Initialize the ports and prepare the blackboard interface.

        Registers all declared input and output ports with the blackboard client, optionally applying custom key
        remappings, and sets the namespace and logger.

        This method must be called before using ``get_input()``, ``_set_output()``, or accessing the
        ``blackboard`` or ``logger`` properties.

        **Port remapping rules**

        1. If a port is *not* in ``port_remappings``, its blackboard key is automatically constructed as
           ``/{subtree_namespace}/{port_name}``.
        2. If a remapped key starts with ``/``, it is treated as an absolute/global key.
        3. If a remapped key does *not* start with ``/``, it is treated as a key relative to the given
           namespace, i.e. ``/{subtree_namespace}/{remapped_key}``.

        **General notes**

        1. The underlying data store is currently the ``py_trees.blackboard``, but this is abstracted.
        2. However, the API of ``PortsMixin`` abstracts from the concept of a blackboard, so the underlying
           implementation could change later.
        3. Think of a "data key" as a generic handle to some shared data storage (like a key in a map),
           which can be remapped to match external system requirements.

        Arguments:
            port_remappings (dict): Optional dictionary mapping port names to custom blackboard keys.
            subtree_namespace (str): Namespace to scope the blackboard client (default: ``"/"``).
            logger: Optional logger-like object with ``debug()``/``info()``/``warning()``/``error()`` methods.
                When ``None``, falls back to ``self.logger`` (the native py_trees logger).

        Raises:
            KeyError: If a port in ``port_remappings`` is not declared in the port definitions.

        This function must be called before using ``get_input``, ``_set_output``, or accessing the
        ``blackboard`` property.
        """
        if port_remappings is None:
            port_remappings = {}

        name = self.name

        # Consistency check: all ports that are remapped must be present in the input or output ports
        for port in port_remappings:
            if port not in self.input_ports() and port not in self.output_ports():
                raise KeyError(f"Port '{port}' is not present in the input or output ports")

        # Use attach_blackboard_client() so the client is visible in self.blackboards
        # (upstream introspection / display).
        self._blackboard_client = self.attach_blackboard_client(name=name, namespace=subtree_namespace)
        self._ports_logger = logger if logger is not None else self.logger
        self._subtree_namespace = subtree_namespace

        # Register all keys for reading/writing that are remapped
        for port, key in port_remappings.items():
            if port in self.input_ports():
                if CONST_PREFIX in key:
                    local_key = self._constant_storage_key(port)
                    # Register write access temporarily to put constant value
                    self._blackboard_client.register_key(key=local_key, access=py_trees.common.Access.WRITE)

                    # Obtaining the initial direct value
                    raw_value = key.split(CONST_PREFIX, 1)[1]
                    # Replacing the DOT_REPLACEMENT with the actual dot (see comment in DOT_REPLACEMENT definition)
                    value = raw_value.replace(DOT_REPLACEMENT, ".")

                    port_type = self.input_ports()[port].data_type
                    try:
                        updated_value = convert_str_to_type(value, port_type, logger=self._ports_logger)
                        self.log_debug(f"Port {port}: Converted const value '{value}' to type {port_type}.")
                    except ValueError as e:
                        raise ValueError(f"Cannot convert Value '{value}' to type {port_type}") from e
                    key = local_key  # Remap to the local key holding the constant value
                    self._blackboard_client.set(key, updated_value)
                # Resolve relative remap targets under the subtree namespace.
                # py_trees.blackboard.Client.register_key() uses remap_to as-is
                # without applying the client's namespace, so relative keys
                # like "transfer" would become the global literal key "/transfer"
                # and collide across sibling subtrees.
                key = py_trees.blackboard.Blackboard.absolute_name(subtree_namespace, key)
                self._blackboard_client.register_key(
                    key=port,
                    access=py_trees.common.Access.READ,
                    required=self.is_port_required(port),
                    remap_to=key,
                )
                abs_port_name = self.blackboard_client.absolute_name(port)
                self.log_debug(
                    f"Port '{port}': Registered blackboard key '{abs_port_name}' for reading [remapped to {key}]."
                )
            elif port in self.output_ports():
                # Resolve relative remap targets under the subtree namespace
                # (see comment in the input branch above for rationale).
                key = py_trees.blackboard.Blackboard.absolute_name(subtree_namespace, key)
                self._blackboard_client.register_key(
                    key=port,
                    access=py_trees.common.Access.WRITE,
                    required=self.is_port_required(port),
                    remap_to=key,
                )
                abs_port_name = self.blackboard_client.absolute_name(port)
                self.log_debug(
                    f"Port '{port}': Registered blackboard key '{abs_port_name}' for writing [remapped to {key}]."
                )

        # Create keys for the ports that are NOT remapped
        for port in self.input_ports():
            if port not in port_remappings:
                storage_key = self._default_port_storage_key(port)
                self.log_debug(f"Port {port}: Registering key '{storage_key}' for reading [default]")
                self._blackboard_client.register_key(
                    key=port,
                    access=py_trees.common.Access.READ,
                    required=self.is_port_required(port),
                    remap_to=storage_key,
                )
        for port in self.output_ports():
            if port not in port_remappings:
                storage_key = self._default_port_storage_key(port)
                self.log_debug(f"Port {port}: Registering key '{storage_key}' for writing [default]")
                self._blackboard_client.register_key(
                    key=port,
                    access=py_trees.common.Access.WRITE,
                    required=self.is_port_required(port),
                    remap_to=storage_key,
                )

    @property
    def subtree_namespace(self) -> str:
        """Return the namespace associated with the subtree, used to scope blackboard keys.

        Raises:
            RuntimeError: If `setup_ports()` has not been called before accessing the namespace.
        """
        if self._subtree_namespace is None:
            raise RuntimeError("PortsMixin.setup_ports() must be called before accessing the subtree_namespace.")
        return self._subtree_namespace

    @property
    def blackboard_client(self) -> py_trees.blackboard.Client:
        """Return the scoped blackboard client.

        Raises:
            RuntimeError: If `setup_ports()` has not been called before accessing the blackboard.
        """
        if self._blackboard_client is None:
            raise RuntimeError("PortsMixin.setup_ports() must be called before accessing the blackboard.")
        return self._blackboard_client

    @property
    def behaviour_class_name(self) -> str:
        """Return the name under which this behavior class is registered.

        This is typically the class name (e.g., "CheckGraspStatus"), but can also be an alias
        used for partial instantiations or custom registrations in a behavior registry.

        Return:
            str: The registered name of this behavior class.
        """
        return self._behaviour_class_name

    def _is_instance_of_type(self, value: Any, expected_type: Any) -> bool:
        """
        Check if a value is an instance of a specific type.

        Extends Python's isinstance() to check for generic types, such as lists.

        Currently this only supports basic types (int, float, etc.) and the generic types Union and list.
        Add additional type support as needed.

        Args:
            value (Any): The value to check.
            expected_type (type): The expected type.

        Returns:
            bool: True if the value is an instance of the expected type, False otherwise.

        Raises:
            NotImplementedError: If type checking for the specific generic type is not implemented.
        """
        origin = typing.get_origin(expected_type)
        args = typing.get_args(expected_type)
        # Handle union types first
        if origin is typing.Union or origin is types.UnionType:  # Need to also check types.UnionType to cover | syntax
            return any(self._is_instance_of_type(value, arg) for arg in args)
        # Handle other generics
        if origin is not None:
            if not isinstance(value, origin):
                return False
            if origin is list and args:
                return all(self._is_instance_of_type(v, args[0]) for v in value)
            raise NotImplementedError(f"Type checking for generic type '{origin}' is not implemented.")
        else:
            return isinstance(value, expected_type)

    def get_logger(self) -> PortsLogger:
        """Return the logger instance.

        Raises:
            RuntimeError: If `setup_ports()` has not been called before accessing the logger.
        """
        if self._ports_logger is None:
            raise RuntimeError(f"{self.name}: PortsMixin.setup_ports() must be called before accessing the logger.")
        return self._ports_logger

    def log(
        self,
        level: LogLevel,
        msg: str,
        return_only: bool = False,
        print_name: bool = True,
    ) -> str:
        """Log a message at the specified severity level and update feedback."""
        assert isinstance(self, py_trees.behaviour.Behaviour), (
            "PortsMixin class needs to derive also from py_trees.behaviour.Behaviour"
        )
        return set_feedback_and_log(
            self,
            name=strip_trailing_uuid4(self.name) if print_name else "",
            level=level,
            message=msg,
            logger=self.get_logger(),
            return_only=return_only,
        )

    def log_debug(self, msg: str, return_only: bool = False, print_name: bool = True) -> str:
        """Log *msg* at DEBUG level (see :meth:`log`)."""
        return self.log(LogLevel.DEBUG, msg, return_only=return_only, print_name=print_name)

    def log_info(self, msg: str, return_only: bool = False, print_name: bool = True) -> str:
        """Log *msg* at INFO level (see :meth:`log`)."""
        return self.log(LogLevel.INFO, msg, return_only=return_only, print_name=print_name)

    def log_warning(self, msg: str, return_only: bool = False, print_name: bool = True) -> str:
        """Log *msg* at WARNING level (see :meth:`log`)."""
        return self.log(LogLevel.WARNING, msg, return_only=return_only, print_name=print_name)

    def log_error(self, msg: str, return_only: bool = False, print_name: bool = True) -> str:
        """Log *msg* at ERROR level (see :meth:`log`)."""
        return self.log(LogLevel.ERROR, msg, return_only=return_only, print_name=print_name)

    def get_input(self, port_name: str, default: Any = None) -> Any:
        """Read the value of the given input port from the blackboard.

        Args:
            port_name (str): The name of the input port to read from the blackboard.
            default (Any): Optional default value to return if the port has no input data.
            If set to `None`, no default is accepted.
        Return:
            Any: The value retrieved from the blackboard for the specified input port or the default.

        Raises:
            KeyError: If the input port name is not defined.
            TypeError: If the retrieved value does not match the expected type.
            NoDataAvailable: If no data is available on the input port and no default is given.
        """
        if port_name not in self.input_ports():
            raise KeyError(f"{self.name}: Input port '{port_name}' not defined.")
        if not self.blackboard_client.is_registered(port_name):
            raise KeyError(f"{self.name}: Input port '{port_name}' is not registered in the blackboard client.")
        # Get the value from the blackboard
        # If the port is not set, return the default value if provided, otherwise raise an error.
        if not self.blackboard_client.exists(port_name):
            if default is not None:
                return default
            raise NoDataAvailable(
                f"{self.name}: Input port '{port_name}' (mapped to "
                f"'{self._get_blackboard_key(port_name)}') has no data available."
            )
        value = self.blackboard_client.get(port_name)

        if value is None:
            raise NotImplementedError("Support for None values has not yet been considered.")
        port_type = self.input_ports()[port_name].data_type
        if not self._is_instance_of_type(value, port_type):
            raise TypeError(f"{self.name}: Value '{value}' is not of type {port_type}, but {type(value)}")
        return value

    def get_last_output(self, port_name: str) -> Any:
        """
        Return the last output which the node wrote at this port.

        Args:
            port_name (str): The name of the output port to read from the blackboard.
        Return:
            Any: The value retrieved from the blackboard for the specified output port.

        Raises:
            KeyError: If the input port name is not defined.
            TypeError: If the value does not match the expected type.
            NoDataAvailable: If no data has (yet) been written to the output port.
        """
        if port_name not in self.output_ports():
            raise KeyError(f"{self.name}: output port '{port_name}' not defined.")
        if not self.blackboard_client.is_registered(port_name):
            raise KeyError(f"{self.name}: output port '{port_name}' is not registered in the blackboard client.")
        if not self.blackboard_client.exists(port_name):
            raise NoDataAvailable(f"{self.name}: output port '{port_name}' has no data available.")
        # Get the value from the blackboard
        value = self.blackboard_client.get(port_name)
        if value is None:
            raise NotImplementedError("Support for explicit None values has not yet been considered.")
        port_type = self.output_ports()[port_name].data_type
        if not self._is_instance_of_type(value, port_type):
            raise TypeError(f"{self.name}: Value '{value}' is not of type {port_type}")
        return value

    def _set_output(self, port_name: str, value: Any) -> None:
        """Write *value* to the given output port on the blackboard.

        Args:
            port_name (str): The name of the output port to write to the blackboard.
            value (Any): The value to set, which must match the expected type of the port.

        Return:
            None

        Raises:
            KeyError: If the output port name is not defined.
            TypeError: If the value does not match the expected type of the port.
        """
        if port_name not in self.output_ports():
            raise KeyError(f"{self.name}: Output port '{port_name}' not defined.")
        port_type = self.output_ports()[port_name].data_type
        if not self._is_instance_of_type(value, port_type):
            raise TypeError(f"{self.name}: Value '{value}' is not of type {port_type}")

        self.blackboard_client.set(port_name, value)

    def reset_port(self, port_name: str) -> None:
        """
        Clear the value stored for a (usually output) port.

        Keeps the key registered (READ/WRITE permissions unaffected), but removes
        the stored value. Intended for use-cases like "new data epoch" where
        downstream nodes should not read stale outputs.

        This will have the effect that subsequent
        `blackboard_client.exists(port_name)` returns False again

        Raises:
            KeyError: if the port is unknown or not registered.
        """
        # We allow resetting either input or output ports, but validate membership
        if (port_name not in self.input_ports()) and (port_name not in self.output_ports()):
            raise KeyError(f"{self.name}: Port '{port_name}' not defined on {self.__class__.__name__}.")

        reset_blackboard_key(self.blackboard_client, port_name, node_name=self.name)

    def reset_all_output_ports(self) -> None:
        """
        Clear all output ports registered on this node.

        Keeps the keys registered (READ/WRITE permissions unaffected), but removes
        the stored values. Intended for use-cases like "new data epoch" where
        downstream nodes should not read stale outputs.

        This will have the effect that subsequent
        `blackboard_client.exists(port_name)` returns False again

        Raises:
            KeyError: if any port is unknown or not registered.
        """
        for port in self.output_ports():
            self.reset_port(port)

    def _get_blackboard_key(self, port_name: str) -> str:
        """Return the blackboard key that the port writes to, considering remappings."""
        remapped_key: str
        try:
            abs_name = self.blackboard_client.absolute_name(port_name)
            remapped_key = self.blackboard_client.__getattribute__("remappings")[abs_name]
        except (KeyError, AttributeError):
            remapped_key = self.blackboard_client.absolute_name(port_name)
        return remapped_key

    def __str__(self) -> str:
        return f"{self.__class__.__name__}=='{self.name}'"

    def __repr__(self) -> str:
        return self.__str__()

    def _scoped_node_name(self) -> str:
        """
        Return a unique node name within the current subtree namespace, suitable for use in blackboard keys.

        Combines the subtree namespace, node name, and node UUID to ensure uniqueness.
        """
        namespace = (self._subtree_namespace or "/").rstrip("/")
        unique_node_name = sanitize_name_for_blackboard_use(f"{self.name}_{self.id}")
        if namespace:
            return f"{namespace}/{unique_node_name}"
        return f"/{unique_node_name}"

    def _default_port_storage_key(self, port_name: str) -> str:
        base = self._scoped_node_name()
        sanitized_port = sanitize_name_for_blackboard_use(port_name)
        return f"{base}/{sanitized_port}"

    def _constant_storage_key(self, port_name: str) -> str:
        return f"{self._default_port_storage_key(port_name)}__const"


class BehaviourWithPorts(PortsMixin, py_trees.behaviour.Behaviour):
    """
    Base class for behaviours with typed input and output ports (see :class:`PortsMixin`).

    Subclassing requirements:

    - Each subclass must implement the ``input_ports`` and ``output_ports`` class methods to specify
      its input and output ports.
    - Each subclass must implement the ``update()`` method to define its behaviour.
    - Other methods from :class:`py_trees.behaviour.Behaviour` may be overridden as needed.

    Example usage::

        class ExampleBehaviour(BehaviourWithPorts):
            @classmethod
            def input_ports(cls):
                return {"input_data": PortInformation(data_type=str, required=True)}

            @classmethod
            def output_ports(cls):
                return {"output_data": PortInformation(data_type=str, required=True)}

            def update(self):
                # Implementation of the behaviour
                ...

    Status semantics for ``update()``:

    - Returning ``FAILURE`` indicates a **technical error** (e.g. service call failures, exceptions)
      that may potentially be handled by nodes such as ``Retry``.
    - If not resolved, a ``FAILURE`` will cause the entire tree to fail.
    - Do **not** use ``FAILURE`` as an expected logical outcome, just so that nodes like ``Retry`` can
      handle those (e.g. when no objects are detected in an image --- that is a valid result, not a failure).
      Instead, indicate such logical outcomes via node outputs (e.g. an empty list of detected objects).
    """

    def __init__(self, name: str, **kwargs: Any) -> None:
        """Initialise the behaviour with *name* (forwarded to :class:`~py_trees.behaviour.Behaviour`)."""
        super().__init__(name=name, **kwargs)

    def update(self) -> py_trees.common.Status:
        """Subclass-defined tick logic. Must return a :class:`~py_trees.common.Status`."""
        raise NotImplementedError("Subclasses must implement update()")

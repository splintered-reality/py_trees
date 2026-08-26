#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/splintered-reality/py_trees/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""Helpers shared between :mod:`py_trees.ports` and :mod:`py_trees.parsers`."""

##############################################################################
# Imports
##############################################################################

import importlib
import inspect
import re
import uuid
from collections.abc import Callable
from enum import Enum
from types import UnionType
from typing import Any, Protocol, Union, get_args, get_origin

import py_trees


class LogLevel(Enum):
    """Severity levels accepted by :class:`PortsLogger`-compatible loggers."""

    DEBUG = "debug"
    INFO = "info"
    WARNING = "warning"
    ERROR = "error"


class PortsLogger(Protocol):
    """Minimal logger interface used by the ports subsystem.

    Any object that provides these four methods is accepted wherever
    the ports code takes an optional ``logger`` parameter.  This is
    satisfied by Python's ``logging.Logger``, py_trees'
    ``py_trees.logging.Logger``, and typical ROS 2 loggers.
    """

    def debug(self, msg: str) -> None:  # noqa: D102
        ...

    def info(self, msg: str) -> None:  # noqa: D102
        ...

    def warning(self, msg: str) -> None:  # noqa: D102
        ...

    def error(self, msg: str) -> None:  # noqa: D102
        ...


class _NoOpLogger:
    """Silent logger used as default when no logger is provided."""

    def debug(self, msg: str) -> None:
        pass

    def info(self, msg: str) -> None:
        pass

    def warning(self, msg: str) -> None:
        pass

    def error(self, msg: str) -> None:
        pass


NOOP_LOGGER = _NoOpLogger()


def _try_bool(value: str) -> bool:
    """Convert common textual boolean values to ``bool``."""
    lowered = value.strip().lower()
    if lowered in ("true", "1", "yes", "on"):
        return True
    if lowered in ("false", "0", "no", "off"):
        return False
    raise ValueError(f"Cannot convert '{value}' to bool")


def _is_enum_type(tp: type) -> bool:
    """Return whether *tp* is an :class:`Enum` subclass."""
    try:
        return inspect.isclass(tp) and issubclass(tp, Enum)
    except TypeError:
        return False


def _convert_to_enum(value: str, enum_type: type[Enum]) -> Enum:
    """Convert *value* to an enum member by name or compatible value."""
    for member in enum_type:
        if member.name.lower() == value.strip().lower():
            return member

    sample = next(iter(enum_type))
    if isinstance(sample.value, str):
        for member in enum_type:
            if member.value == value:
                return member
    elif isinstance(sample.value, int):
        try:
            ivalue = int(value)
        except ValueError:
            pass
        else:
            for member in enum_type:
                if member.value == ivalue:
                    return member

    raise ValueError(f"Cannot convert '{value}' to enum {enum_type.__name__}")


def str_to_type(type_spec: str) -> type:
    """
    Resolve a fully-qualified or built-in type name into the actual type.

    A fully-qualified name includes the module path, e.g. ``'package.module.Type'``.
    Built-in types can be specified by their simple name, e.g. ``'int'``, ``'str'``, ``'list'``, etc.
    Raises :class:`ValueError` if the type cannot be resolved.
    """
    module_name, partition, type_name = type_spec.rpartition(".")
    if not partition:
        builtins_dict = vars(__import__("builtins"))
        candidate = builtins_dict.get(type_spec)
        if isinstance(candidate, type):
            return candidate

    if not module_name:
        raise ValueError("Type specification must include module path, e.g. 'package.module.Type'.")
    module = importlib.import_module(module_name)
    resolved = getattr(module, type_name)
    if not isinstance(resolved, type):
        raise TypeError(f"Resolved '{type_spec}' to non-type object {resolved!r}.")
    return resolved


def _convert_simple(value: str, target_type: type) -> tuple[bool, Any]:
    """Convert *value* to a supported scalar type, or return it unchanged."""
    if target_type is str:
        return True, value
    if target_type is bool:
        return True, _try_bool(value)
    if target_type is int:
        return True, int(value)
    if target_type is float:
        return True, float(value)
    if _is_enum_type(target_type):
        return True, _convert_to_enum(value, target_type)  # type: ignore
    # Fallback: leave as string
    return False, value


def convert_str_to_type(
    value: str, target_type: type | UnionType, logger: PortsLogger | None = None
) -> tuple[bool, Any]:
    """Convert a string *value* to *target_type* (handles unions, list, tuple, enum)."""
    if logger is None:
        logger = NOOP_LOGGER

    # If target type is `type`, the result needs to be parsed to a type. Example: 'float' -> <class 'float'>
    if target_type is type:
        try:
            return True, str_to_type(value)
        except (ValueError, ImportError, AttributeError, TypeError):
            pass  # Fall back to try to parse it further down

    origin = get_origin(target_type)

    if isinstance(target_type, type) and issubclass(target_type, list):
        origin = list

    if origin is None:
        if not isinstance(target_type, type):
            # Not a runtime type (e.g. typing.Any on Python 3.10, or a stringised annotation):
            # there is nothing we can convert to, so report failure and keep the string.
            return False, value
        return _convert_simple(value, target_type)

    if origin is Union or origin is UnionType:
        args = get_args(target_type)
        has_none = any(a is type(None) for a in args)  # noqa: E721
        lowered = value.strip().lower()
        if has_none and (lowered == "" or lowered == "none" or lowered == "null"):
            return True, None
        for arg in args:
            if arg is type(None):  # noqa: E721
                continue
            try:
                arg_success, arg_value = convert_str_to_type(value, arg, logger)
            except Exception:
                continue
            # A member reporting failure is no better than one that raised: try the next one.
            if arg_success:
                return True, arg_value
        return False, value

    if origin is list:
        (inner_type,) = get_args(target_type) or (str,)
        parts = [p.strip() for p in re.split(r"[,;]", value)] if value.strip() else []
        try:
            results = [convert_str_to_type(p, inner_type, logger) for p in parts]
            successes = [s for s, _ in results]
            return all(successes), [r for _, r in results]
        except Exception:
            return False, [p.strip() for p in parts]

    if origin is tuple:
        inner_types = get_args(target_type)
        parts = [p.strip() for p in re.split(r"[,;]", value)]
        converted = []
        for i, p in enumerate(parts):
            t = inner_types[i] if i < len(inner_types) else str
            try:
                converted.append(convert_str_to_type(p, t, logger))
            except Exception:
                converted.append((False, p))
        successes = [s for s, _ in converted]
        return all(successes), tuple(r for _, r in converted)

    return False, value


def is_instance_of_type(value: Any, expected_type: Any) -> bool:
    """
    Check if a value is an instance of a specific type.

    Extends Python's isinstance() to check for generic types, such as lists.

    Currently this only supports basic types (int, float, etc.) and the generic types Union and list.
    Add additional type support as needed.

    Args:
        value (Any): The value to check.
        expected_type (Any): The expected type.

    Returns:
        bool: True if the value is an instance of the expected type, False otherwise.

    Raises:
        NotImplementedError: If type checking for the specific generic type is not implemented.
    """
    origin = get_origin(expected_type)
    args = get_args(expected_type)
    # Handle union types first
    if origin is Union or origin is UnionType:  # Need to also check types.UnionType to cover | syntax
        return any(is_instance_of_type(value, arg) for arg in args)
    # Handle other generics
    if origin is not None:
        if not isinstance(value, origin):
            return False
        if origin is list and args:
            return all(is_instance_of_type(v, args[0]) for v in value)
        raise NotImplementedError(f"Type checking for generic type '{origin}' is not implemented.")
    else:
        return isinstance(value, expected_type)


def collect_type_hints(constructor: Callable) -> dict[str, Any]:
    """Collect parameter type hints from a constructor.

    When ``constructor`` is a class, the method resolution order (MRO) is walked so
    that a type hint declared on a parent's ``__init__`` is still found when the
    subclass forwards ``**kwargs`` to ``super().__init__()``, and therefore does
    not declare the parameter itself, or declares it without an annotation.

    The most-derived annotation for a given parameter name wins: walking the MRO
    from subclass to base and only recording the first hint seen per parameter.

    Args:
        constructor: A class (whose ``__init__`` chain is inspected) or a callable.

    Returns:
        dict[str, Any]: Mapping of parameter name to its type annotation. Parameters
            without an annotation anywhere in the hierarchy are omitted, as is
            ``self`` and any ``*args`` catch-all. A ``**kwargs`` catch-all is kept
            (typically under the key ``"kwargs"``) so callers can fall back to it
            for keys that don't have their own individual type hint.
    """
    classes = constructor.__mro__ if inspect.isclass(constructor) else (constructor,)

    hints: dict[str, Any] = {}
    for klass in classes:
        func = klass.__init__ if inspect.isclass(klass) else klass
        try:
            sig = inspect.signature(func)
        except (TypeError, ValueError):
            # Built-ins (e.g. object.__init__ on some interpreters) may not be
            # introspectable; just skip them and keep walking the hierarchy.
            continue

        for pname, param in sig.parameters.items():
            if pname == "self":
                continue
            if param.kind is inspect.Parameter.VAR_POSITIONAL:
                continue
            if param.annotation is inspect._empty:
                continue
            # The first hint found wins => the most-derived class in the MRO.
            if pname in hints:
                continue
            hints[pname] = param.annotation

    return hints


def apply_type_hints(
    constructor: Callable,
    kwargs: dict[str, Any],
    logger: PortsLogger | None = None,
    ignore: set[str] | None = None,
) -> tuple[bool, dict[str, Any]]:
    """
    Convert XML string kwargs into hinted types from the constructor signature.

    Keys that are in the `ignore` set will be kept as-is.

    - If `constructor` is a class, its `__init__` is inspected (excluding `self`).
    - Only parameters that have type annotations are converted.
    - A key with no individual type hint falls back to the constructor's `**kwargs`
      annotation, if any (e.g. dynamic keys consumed by a `**kwargs: SomeType` catch-all).
      The catch-all is looked up by the conventional name `kwargs`, so one declared under
      another name (e.g. `**options`) provides no fallback.
    - An unconstrained target type (`Any`, `object`) counts as a success with the string kept,
      since any value satisfies it.
    - On conversion failure, the original string is preserved and a warning is printed.
      The function return indicates that there was a failure in one of the values.

    Returns:
        tuple[bool, dict[str, Any]]: Success of conversion (True if all values were successfully
            converted, False if any conversion failed, a warning was also printed then), and the
            converted dictionary.
    """
    if ignore is None:
        ignore = set()
    if logger is None:
        logger = NOOP_LOGGER

    hints = collect_type_hints(constructor)

    converted: dict[str, Any] = {}
    success = True
    for k, v in kwargs.items():
        if k in ignore:
            converted[k] = v
            continue

        # Obtain the type hint with fallback to the constructor's **kwargs annotation, if any.
        tp = hints.get(k, hints.get("kwargs"))
        # Default behavior: keep the original value.
        # Warning will be printed at the end of this loop if it failed to be converted.
        converted[k] = v

        # No type hint given: keep the original value
        if tp is None:
            logger.warning(f"Skipping conversion for '{k}': no type hint available.")
            success = False
            continue

        # Target type hint exists. Handle conversion, if needed.
        if tp is str:
            # Target type is already a string: no need to do anything.
            continue

        if tp is Any or tp is object:
            # The target puts no constraint on the value, so string is already fine.
            continue

        # Not a string: if the target is already of the correct type, we can just keep it as-is.
        if not isinstance(v, str):
            if tp is not type(v):
                logger.warning(
                    f"Type {type(v)} is not a string which can be converted, and not of the required "
                    f"target type {tp}. Keeping the string and leaving conversion to the constructor."
                )
                success = False
            # Keep as-is.
            continue

        try:
            conversion_success, converted[k] = convert_str_to_type(v, tp, logger)
            if not conversion_success:
                logger.warning(f"Failed to convert '{k}: {v}' to type '{tp}'. Result: '{converted[k]}'.")
                success = False
        except Exception as e:
            # Resolving a type by name can raise anything the target module raises on import.
            logger.warning(f"Failed to convert '{k}: {v}' to type '{tp}': {e}")
            success = False

    return success, converted


def reset_blackboard_key(
    blackboard_client: "py_trees.blackboard.Client",
    key_name: str,
    node_name: str = "unknown",
) -> None:
    """Clear the stored value for *key_name* via its registered client."""
    if not blackboard_client.is_registered(key_name):
        raise KeyError(f"{node_name}: Port '{key_name}' is not registered in the blackboard client.")

    try:
        blackboard_client.unset(key_name)
    except Exception as e:
        raise RuntimeError(f"{node_name}: Unable to reset port '{key_name}'.") from e


def uuid4_regex(at_end: bool = False) -> str:
    """Return a regex fragment matching a UUID4 string."""
    return r"((_)?[a-f0-9\-]{36})" + "$" if at_end else ""


def strip_trailing_uuid4(name: str) -> str:
    """Remove a trailing UUID4 suffix from *name*."""
    return re.sub(f"{uuid4_regex(at_end=True)}", "", name, flags=re.IGNORECASE)


def get_base_name(name: str, strip_uuid: bool = False) -> str:
    """
    Extract the base name from a fully-qualified name.

    Args:
        name: The fully-qualified name (e.g., "namespace1.namespace2.NodeName").
        strip_uuid: Strip UUID suffix generated by generate_node_name().
    """
    if strip_uuid:
        name = re.sub(f"{uuid4_regex(at_end=True)}", "", name, flags=re.IGNORECASE)
    if "." in name:
        return name.rsplit(".", 1)[1]
    return name


def generate_node_name(
    explicit_name: str | None,
    general_name: str = "",
    prefix: str = "",
    no_uuid: bool = False,
) -> str:
    """
    Generate a node name.

    Args:
        explicit_name: Optional explicit name set by user.
        general_name: Fallback node category name.
        prefix: Optional dot-separated parent prefix.
        no_uuid: If True, do not append UUID when using general_name fallback.
    """
    use_name = explicit_name
    if not use_name:
        use_name = general_name
        if not no_uuid:
            use_name += "_" + str(uuid.uuid4())
    prefix = prefix + ("." if use_name else "") if prefix else ""
    return prefix + use_name


def sanitize_name_for_blackboard_use(component: str, extra_allowed_chars: str = "") -> str:
    """Replace characters py_trees treats as separators with underscores."""
    safe_extra = re.escape(extra_allowed_chars)
    expr_str = f"[^A-Za-z0-9_-{safe_extra}]"
    return re.sub(expr_str, "_", component)


def set_feedback_and_log(
    behaviour: py_trees.behaviour.Behaviour,
    *,
    name: str,
    message: str,
    level: LogLevel = LogLevel.INFO,
    logger: PortsLogger | None = None,
    return_only: bool = False,
) -> str:
    """Format *message*, update *behaviour.feedback_message*, and log at *level*."""
    if logger is None:
        logger = NOOP_LOGGER
    message = str(message)
    formatted = f"{name}: {message}" if name else message
    if return_only:
        return formatted

    if level != LogLevel.DEBUG:
        behaviour.feedback_message = formatted

    if logger is not None:
        log_fn = getattr(logger, level.value, None)
        if callable(log_fn):
            log_fn(formatted)
    return formatted


def find_node_by_name(
    node: py_trees.behaviour.Behaviour,
    name: str,
    strip_prefix: bool = False,
    strip_uuid: bool = False,
    find_all: bool = False,
) -> py_trees.behaviour.Behaviour | list[py_trees.behaviour.Behaviour] | None:
    """Find a node (or nodes) by name in a behavior tree."""
    if find_all:
        results: list[py_trees.behaviour.Behaviour] = []
        _find_node_by_name_recursive(node, name, strip_prefix, strip_uuid, results)
        return results

    result_list: list[py_trees.behaviour.Behaviour] = []
    _find_node_by_name_recursive(node, name, strip_prefix, strip_uuid, result_list, stop_at_first=True)
    return result_list[0] if result_list else None


def _find_node_by_name_recursive(
    node: py_trees.behaviour.Behaviour,
    name: str,
    strip_prefix: bool,
    strip_uuid: bool,
    results: list,
    stop_at_first: bool = False,
) -> bool:
    """Depth-first implementation for :func:`find_node_by_name`."""
    mod_node_name = get_base_name(node.name, strip_uuid=strip_uuid) if strip_prefix else node.name
    if mod_node_name == name:
        results.append(node)
        if stop_at_first:
            return True

    if hasattr(node, "children") and node.children:
        for c in node.children:
            if _find_node_by_name_recursive(c, name, strip_prefix, strip_uuid, results, stop_at_first):
                return True
    elif hasattr(node, "child") and node.child:
        if _find_node_by_name_recursive(node.child, name, strip_prefix, strip_uuid, results, stop_at_first):  # type: ignore
            return True
    elif (
        hasattr(node, "decorated")
        and node.decorated
        and _find_node_by_name_recursive(node.decorated, name, strip_prefix, strip_uuid, results, stop_at_first)  # type: ignore
    ):
        return True

    return False


def find_node_by_class(node: py_trees.behaviour.Behaviour, class_: type) -> Any:
    """Recursively search a tree for the first node instance of a class."""
    if isinstance(node, class_):
        return node
    if hasattr(node, "children"):
        for child in node.children:
            try:
                return find_node_by_class(child, class_)
            except ValueError:
                continue
    raise ValueError(f"Node of class {class_.__name__} not found in the tree")

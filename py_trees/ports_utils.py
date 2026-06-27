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


def _convert_simple(value: str, target_type: type) -> Any:
    """Convert *value* to a supported scalar type, or return it unchanged."""
    if target_type is str:
        return value
    if target_type is bool:
        return _try_bool(value)
    if target_type is int:
        return int(value)
    if target_type is float:
        return float(value)
    if _is_enum_type(target_type):
        return _convert_to_enum(value, target_type)  # type: ignore
    return value


def convert_str_to_type(value: str, target_type: type | UnionType, logger: PortsLogger | None = None) -> Any:
    """Convert a string *value* to *target_type* (handles unions, list, tuple, enum)."""
    if logger is None:
        logger = NOOP_LOGGER
    origin = get_origin(target_type)

    if isinstance(target_type, type) and issubclass(target_type, list):
        origin = list

    if origin is None:
        assert isinstance(target_type, type)
        return _convert_simple(value, target_type)

    if origin is Union or origin is UnionType:
        args = get_args(target_type)
        has_none = any(a is type(None) for a in args)  # noqa: E721
        lowered = value.strip().lower()
        if has_none and (lowered == "" or lowered == "none" or lowered == "null"):
            return None
        for arg in args:
            if arg is type(None):  # noqa: E721
                continue
            try:
                return convert_str_to_type(value, arg, logger)
            except Exception:
                continue
        return value

    if origin is list:
        (inner_type,) = get_args(target_type) or (str,)
        parts = [p.strip() for p in re.split(r"[,;]", value)] if value.strip() else []
        try:
            return [convert_str_to_type(p, inner_type, logger) for p in parts]
        except Exception:
            return [p.strip() for p in parts]

    if origin is tuple:
        inner_types = get_args(target_type)
        parts = [p.strip() for p in re.split(r"[,;]", value)]
        converted = []
        for i, p in enumerate(parts):
            t = inner_types[i] if i < len(inner_types) else str
            try:
                converted.append(convert_str_to_type(p, t, logger))
            except Exception:
                converted.append(p)
        return tuple(converted)

    return value


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
            without an annotation anywhere in the hierarchy are omitted, as are
            ``self`` and any ``*args``/``**kwargs`` catch-alls.
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
            if param.kind in (inspect.Parameter.VAR_POSITIONAL, inspect.Parameter.VAR_KEYWORD):
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
) -> tuple[dict[str, Any], bool]:
    """
    Convert XML string kwargs into hinted types from the constructor signature.

    Keys that are in the `ignore` set will be kept as-is.

    - If `constructor` is a class, its `__init__` is inspected (excluding `self`).
    - Only parameters that have type annotations are converted.
    - On conversion failure, the original string is preserved and a warning is printed.
      The function return indicates that there was a failure in one of the values.

    Returns:
        tuple[dict[str, Any], bool]: The converted dictionary and a flag set to False if one of
           the values was attempted bo convert but it failed (a warning was also printed then).
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

        tp = hints.get(k)
        # Default behavior: keep the original value.
        # Warning will be printed at the end of this loop if it is not overwritten.
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
            converted[k] = convert_str_to_type(v, tp, logger)
        except ValueError as e:
            logger.warning(f"Failed to convert '{k}: {v}' to type '{tp}': {e}")
            success = False
            continue

        if converted[k] == v:
            logger.warning(f"Failed to convert '{k}: {v}' to type '{tp}'. Preserved original value.")
            success = False

    return converted, success


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

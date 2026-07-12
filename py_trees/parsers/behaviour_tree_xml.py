#!/usr/bin/env python
#
# License: BSD
#   https://raw.githubusercontent.com/splintered-reality/py_trees/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################

"""
XML parser for the BehaviorTree format.

.. note::

   The parser is experimental and its API may change between releases.

This module provides a parser for the BehaviorTree XML format, used to construct behaviour trees with key remapping
and subtree instantiation.

Overview
--------

The parser recursively builds a behaviour tree from an XML file, using a remapping table to track key assignments
and substitutions. The remapping table is a dictionary that maps keys (referenced in curly braces in the XML,
e.g. ``{key}``) to either their absolute paths (e.g. ``/some/key``) or to other keys
(e.g. ``{other_key}``), which are then further resolved to absolute paths.

In the end, all keys in the tree map to absolute paths which can be used to address the value in a map,
blackboard, or similar structure.

Remapping table example::

    {
        "absolute_value": "/absolute/path",
        "curly_reference": "{absolute_value}",
    }

Keys are resolved recursively until an absolute path is found. This allows flexible wiring of data flow between
nodes and subtrees.

Subtree templates and instantiation
-----------------------------------

Subtrees are defined as ``<BehaviorTree ID="...">`` elements in the XML. These act as templates, which can be
instantiated elsewhere in the tree using a ``<SubTree>`` tag. When a subtree is instantiated, the parser:

- Makes a local remapping table by applying any remappings specified in the ``<SubTree>`` tag.
- Recursively parses the referenced subtree template, using the updated remapping table and a new namespace.

Example subtree template:

.. code-block:: xml

    <BehaviorTree ID="MySubtree">
        <Sequence>
            <Reader name="MyReader" input="{input_key}" />
            <Writer name="MyInternalWriter" output="{transfer_key}" />
            <Reader name="MyInternalReader" input="{transfer_key}" />
        </Sequence>
    </BehaviorTree>

Example main tree instantiating a subtree:

.. code-block:: xml

    <BehaviorTree ID="MainTree">
        <Writer output="{some_key}" name="WriterMain" />
        <SubTree ID="MySubtree" name="Subtree1" input_key="{some_key}"/>
    </BehaviorTree>

This example will map ``{some_key}`` to ``/some_key`` in the remapping table (the root namespace is ``/``),
and when the ``MySubtree`` is instantiated, it will create a new namespace ``/Subtree1`` where:

- ``{input_key}`` resolves to ``/some_key``
- ``{transfer_key}`` (which is not remapped in the ``SubTree`` tag) resolves to
  ``/Subtree1/transfer_key`` --- so a new key is created for the subtree.

A good documentation of remapping and subtrees can be found on the
`BT.CPP documentation <https://www.behaviortree.dev/docs/tutorial-basics/tutorial_06_subtree_ports>`_.

Parsing walkthrough
-------------------

Given the above example, parsing proceeds as follows:

1. The parser starts at the ``MainTree`` with an empty remapping table and namespace ``/``.
2. It encounters the ``Writer`` node, which uses ``{some_key}``. Since this key is new, it is mapped to
   ``/some_key`` in the remapping table.
3. The ``SubTree`` node is encountered. The parser:

   - Copies the current remapping table.
   - Adds ``input_key -> {some_key}`` to the remapping table for the subtree.
   - Sets the namespace to ``/Subtree1``.
   - Recursively parses the ``MySubtree`` template.

4. Inside ``MySubtree``:

   - The ``Reader`` node uses ``{input_key}``, which resolves (via remapping) to ``/some_key``.
   - The ``Writer`` node uses ``{transfer_key}``, which is new, so it is mapped to ``/Subtree1/transfer_key``.
   - The second ``Reader`` node uses ``{transfer_key}``, which now resolves to ``/Subtree1/transfer_key``.

Key concepts
------------

- **Remapping table**: Tracks how keys in curly braces are resolved to absolute paths or to other keys.
- **Namespace**: Each subtree instantiation gets its own namespace, ensuring keys are scoped and do not collide.
- **Subtree instantiation**: Subtrees are templates; instantiating them is like copy-pasting their structure,
  but with remapped keys and a new namespace.

For more details, see the code and the accompanying tests in ``test_ports_xml_parser.py``.
"""

import functools
import inspect
import os
import re
import uuid
import xml.etree.ElementTree as ET
from copy import deepcopy
from typing import Any

import py_trees
from py_trees.ports import CONST_PREFIX, DOT_REPLACEMENT, PortsMixin, get_ports_registry
from py_trees.ports_utils import (
    NOOP_LOGGER,
    PortsLogger,
    apply_type_hints,
    generate_node_name,
)

# Helper: parse curly-brace keys
CURLY_PATTERN = re.compile(r"^{(.+)}$")

# All composite or decorator tags which can have children and have ports (case-insensitive)
DECORATOR_NODES = {
    name.lower(): obj
    for name, obj in inspect.getmembers(py_trees.decorators, inspect.isclass)
    if obj.__module__ == py_trees.decorators.__name__
    and issubclass(obj, py_trees.decorators.Decorator)
    and obj is not py_trees.decorators.Decorator
}

# All composite or decorator tags which can have children (case-insensitive)
COMPOSITE_NODES_TAGS = {"sequence", "selector", "fallback", "parallel"}
PARENT_NODES_TAGS = set(DECORATOR_NODES) | COMPOSITE_NODES_TAGS


class XMLParserError(Exception):
    """Exception raised due to XML parsing failure."""


def build_bt_index(root: ET.Element) -> dict[str, ET.Element]:
    """Return ``{ID: element}`` for every ``<BehaviorTree>`` child of *root*."""
    return {bt.attrib["ID"]: bt for bt in root.findall("BehaviorTree")}


def is_key(value: str) -> re.Match | None:
    """Return a regex match if *value* is a ``{key}`` reference."""
    open_braces = value.count("{")
    close_braces = value.count("}")
    if open_braces == 0 and close_braces == 0:
        return None
    if open_braces != 1 or close_braces != 1:
        raise ValueError(f"Malformed key reference '{value}'")

    match = CURLY_PATTERN.match(value)
    if not match:
        raise ValueError(f"Malformed key reference '{value}'")
    return match


def get_key_name(value: str) -> str:
    """Extract the key name from a ``{key}`` reference string."""
    match = is_key(value)
    assert match, f"Key '{value}' is not a valid key"
    return str(match.group(1))


def resolve_key_remapping(key: str, remapping_table: dict[str, str]) -> str:
    """
    Recursively resolve a key through the remapping table until it is not a curly-brace key.

    Cyclic remappings are explicitly checked and will raise a RuntimeError if detected.

    Args:
        key: Input key. Expected to be either a curly-brace key or an absolute key.
        remapping_table: Maps logical keys to absolute keys.

    Returns:
        str: The resolved absolute key.

    Raises:
        ValueError: If the key can't be properly resolved.
        RuntimeError: If a cyclic remapping is detected.
    """
    visited = set()
    while True:
        match = is_key(key)
        if not match:
            # If not a curly-brace key, it must be an absolute key. Check that this is the case (raise an error if not)
            # and then return the key.
            if not key.startswith("/"):
                raise ValueError(f"Key '{key}' is not an absolute key")
            return key
        inner = match.group(1)
        if inner in visited:
            raise RuntimeError(f"Cyclic remapping detected for key: {key}")
        visited.add(inner)
        if inner in remapping_table:
            key = remapping_table[inner]
        else:
            # If not found, this is an error. If there is a remapping to a key specified,
            # then that key must be in the remapping table - reason: a key, e.g. `{my_key}`,
            # will resolve to an absolute path like /subtree1/my_key (the absolute path which includes
            # the subtree namespace must be in the remapping table, otherwise the information on the subtree
            # namespace is lost).
            raise ValueError(f"Key '{inner}' not found in remapping table")


def resolve_direct_value_remapping(key: str, remapping_table: dict[str, str]) -> str:
    """
    Obtain a direct value from the remapping table.

    Args:
        key: The direct value.
        remapping_table: A mapping of prefixed keys to their resolved values.

    Returns:
        The resolved value from the remapping table.

    Raises:
        ValueError: If the prefixed key is not found in the remapping table or is of invalid format.
    """
    if key.startswith("/"):
        raise ValueError(f"Key '{key}' is of invalid format")

    encoded_key = key.replace(".", DOT_REPLACEMENT)
    key = f"{CONST_PREFIX}{encoded_key}"

    if key in remapping_table:
        key = remapping_table[key]
    else:
        # If not found, this is an error.
        raise ValueError(f"Key '{key}' not found in remapping table")
    return key


def get_absolute_reference(value: str, subtree_namespace: str) -> str:
    """
    Get the absolute path of a value (e.g. a key) by prepending the namespace.

    Args:
        value (str): The value to make absolute.
        subtree_namespace (str): The current subtree namespace.

    Returns:
        str: The absolute reference to the value.
    """
    return subtree_namespace.rstrip("/") + "/" + value


def get_class_from_registry(class_name: str, node_registry: dict) -> type["PortsMixin"]:
    """
    Get the class from the node_registry dictionary, ensuring it is a subclass of :class:`PortsMixin`.

    Args:
        class_name (str): The name of the class to look up.
        node_registry (dict): A dictionary mapping class names to class constructors or partial callables,
            e.g. ``{"Producer": Producer, "Consumer": partial(Consumer, name=name)}``.

    Returns:
        The class (subclass of :class:`PortsMixin`).

    Raises:
        KeyError: If the class name is not found in the node_registry.
        TypeError: If the entry is not a class or partial callable,
            or if the class is not a subclass of :class:`PortsMixin`.
    """
    if class_name not in node_registry:
        raise KeyError(f"Class name '{class_name}' not found in node_registry: {node_registry}")
    entry = node_registry[class_name]

    # Case 1: Direct class reference
    if inspect.isclass(entry):
        cls = entry
    elif isinstance(entry, functools.partial) and inspect.isclass(entry.func):
        cls = entry.func
    else:
        raise TypeError(f"Unsupported entry type for '{class_name}': {type(entry)}")

    # Validate subclass
    if not issubclass(cls, PortsMixin):
        raise TypeError(f"{cls.__name__} is not a subclass of PortsMixin")

    return cls


def parse_behaviour_tree_xml(
    xml_file: str,
    main_tree_id: str | None = None,
    node_registry: dict | str = "auto",
    logger: PortsLogger | None = None,
    search_paths: list[str] | None = None,
) -> py_trees.behaviour.Behaviour:
    """
    Parse the XML file and build the behavior tree.

    Supports simple top-level imports via:
        <Import src="other.xml"/>
        <Include file="other.xml"/>

    The import pre-pass inlines all <BehaviorTree> elements from the referenced XMLs
    into the current document. If any imported BehaviorTree ID already exists, a
    ValueError is raised.

    Node classes are resolved from ``node_registry``, which is either:

    * ``"auto"`` (the default) --- resolve every XML tag from the global registry of
      auto-registered :class:`~py_trees.ports.PortsMixin` subclasses. Importing your
      node classes is enough to make them usable; no explicit mapping is needed.
    * a ``dict`` mapping tag -> class / callable --- use *exactly* this mapping and
      ignore the auto-registry. Use this to inject dependencies via
      ``functools.partial`` (e.g. ``{"Wait": partial(Wait, factory=f)}``), to alias
      tags, or to sandbox the parser to a fixed set of classes.

    To combine auto-registration with a few explicit entries, build the dict on top
    of the auto-registry::

        node_registry={**py_trees.ports.get_ports_registry(), "Wait": partial(Wait, factory=f)}

    Args:
        xml_file (str): Path to the main XML file.
        main_tree_id (str | None): ID of the tree to execute; if None, read from 'main_tree_to_execute'.
        node_registry (dict | str): ``"auto"`` to resolve from the auto-registry, or a
            ``{tag: class/partial}`` dict to use exclusively. Defaults to ``"auto"``.
        logger (PortsLogger | None): Optional logger (NoOp if None).
        search_paths (list[str] | None): Optional extra directories to resolve imports.

    Returns:
        The root py_trees.behaviour.Behaviour for the requested tree.

    Raises:
        ValueError: If ``node_registry`` is a string other than ``"auto"``, if no node
            classes are available, or the main BehaviorTree ID is not found.
        TypeError: If ``node_registry`` is neither a dict nor a string.
        FileNotFoundError / RuntimeError: From the import pre-pass if relevant.
    """
    if logger is None:
        logger = NOOP_LOGGER
    if isinstance(node_registry, str):
        if node_registry != "auto":
            raise ValueError(f"node_registry string must be 'auto', got {node_registry!r}.")
        node_registry = get_ports_registry()
    elif isinstance(node_registry, dict):
        node_registry = dict(node_registry)
    else:
        raise TypeError(
            "node_registry must be a dict (tag -> class/partial) or the string "
            f"'auto', got {type(node_registry).__name__}."
        )
    if not node_registry:
        raise ValueError(
            "No node classes available to the parser: pass node_registry='auto' "
            "with your PortsMixin subclasses imported, or an explicit {tag: class} dict."
        )

    xml_tree = ET.parse(xml_file)
    root = xml_tree.getroot()

    # Load and inline any imports *before* building the index
    _inline_imports_into_root(
        root=root,
        current_file=os.path.abspath(xml_file),
        logger=logger,
        search_paths=search_paths,
    )

    if main_tree_id is None:
        main_tree_id = root.attrib.get("main_tree_to_execute")

    # Pretty-print the whole of the XML tree after imports
    logger.debug(ET.tostring(root, encoding="unicode"))

    bt_index = build_bt_index(root)
    if main_tree_id not in bt_index:
        raise ValueError(f"BehaviorTree with ID '{main_tree_id}' not found.")
    logger.debug(f"[DEBUG] Starting parse of main tree ID='{main_tree_id}'")
    bt_elem = bt_index[main_tree_id]

    tree = build_tree_from_xml(
        bt_elem,
        remapping_table={},
        node_registry=node_registry,
        bt_index=bt_index,
        logger=logger,
        subtree_namespace="/",
        parent_names_str="",
    )
    # Consistency check: traverse the whole tree and check if any nodes have duplicate names.
    seen_names = set()
    for node in tree.iterate():
        logger.debug(f"Node in tree: {node.name} ({node.__class__.__name__})")
        if node.name in seen_names:
            raise ValueError(
                f"Duplicate node name found: {node.name}. Mitigate by assigning explicit names to parent tags."
            )
        seen_names.add(node.name)
    return tree


def add_new_key_to_remapping_table(value: str, remapping_table: dict[str, str], subtree_namespace: str) -> None:
    """
    Add the value to the remapping table, **if** it is a key itself.

    If we encounter a remapping in a ``SubTree`` or ``PortsMixin``-derived tag,
    e.g. `remapped_key={other_key}` and the *value* (i.e. {other_key}) is a key itself,
    and this key is *not* yet in the remapping table, then it means that in the
    current subtree namespace, we have encountered this key for the first time.

    Example:
    ```
    <SubTree ID="subtree1" in="{other_key}"/>
    ```

    The first time we parse a tag which has a key in the assigned value, and the key (i.e. `{other_key}`)
    is not yet in the remapping table, it needs to be added, within the scope of the subtree namespace.

    Args:
        value (str): The value of the remapping, e.g. `{other_key}` in the statement `remapped_key={other_key}`.
        remapping_table (dict[str, str]): The remapping table.
        subtree_namespace (str): The current subtree namespace.
    """
    if is_key(value):
        key_name = get_key_name(value)
    # If it's not a key, then it must be a direct value.
    else:
        encoded_value = value.replace(".", DOT_REPLACEMENT)
        key_name = f"{CONST_PREFIX}{encoded_value}"

    if key_name not in remapping_table:
        scoped_key = get_absolute_reference(key_name, subtree_namespace)
        remapping_table[key_name] = scoped_key


def build_subtree_remapping(
    elem: ET.Element,
    remapping_table: dict[str, str],
    parent_namespace: str,
    logger: PortsLogger = NOOP_LOGGER,
) -> dict[str, str]:
    """
    Process the <SubTree> XML element.

    The remapping table is updated to include the remappings from the <subtreeplus> or <subtree> element.
    The subtree is then instantiated with the new remapping table.

    Args:
        elem: The <SubTree> (or <SubTreePlus>) XML element.
        remapping_table: Parent remapping table (logical name -> absolute key).
        parent_namespace: Absolute namespace of the parent tree.
        logger: Optional logger.

    Returns:
        dict[str, str]: The local remapping for this subtree's ports.

    Raises:
        ValueError: If a referenced key is missing or a value can't be resolved.
        RuntimeError: If cyclic remapping is detected during resolution.
    """
    # Add new keys that appear in the values of an attribute of this SubTree element to the remapping table.
    # Example: `<SubTree ID="subtree1" in="{other_key}" />` - if `other_key` is a key, and it has been
    # encountered during parsing for the first time, it will be added to the remapping table with its absolute path
    # within the parent tree namespace (e.g. `/other_key`), so that it can be resolved when parsing the subtree.
    # We need to do that in order for the resolving (call to `resolve_*_remapping()` below) of the keys to work
    # correctly.
    for k, v in elem.attrib.items():
        if k in ("ID", "name"):
            continue
        logger.debug(f"Checking to add new key for SubTree attribute: {k} -> {v}")
        add_new_key_to_remapping_table(v, remapping_table=remapping_table, subtree_namespace=parent_namespace)
    logger.debug(f"Updated remapping table: {remapping_table}")

    # Build the new remapping table for this subtree. We build a new remapping table because we need to ensure that
    # *only* the keys that are explicitly remapped in the <SubTree> element are included. The parent remapping table
    # may already include keys with the same name that are to be treated as local keys in the subtree namespace.
    new_remapping = {}

    # Add keys from this SubTree element to the local remapping table.
    # If the value is a key and it is already in the remapping table, we resolve it to its absolute path.
    # Example: `<SubTree ID="subtree1" in="{other_key}" />` - we need resolve {other_key} to its absolute
    # path (using the parent remapping) and then add `in -> resolved({other_key})` to the new remapping table.
    for k, v in elem.attrib.items():
        if k in ("ID", "name"):
            continue
        logger.debug(f"Processing SubTree attribute: {k} -> {v}")
        if is_key(v):
            if get_key_name(v) not in remapping_table:
                raise ValueError(f"Key {v} not found in remapping table")
            # If the value is a key and it is in the remapping table,
            # we resolve it to its absolute path.
            v = resolve_key_remapping(v, remapping_table)
            logger.debug(f"[Key] Adding remapping from parent remapping table: {k} -> {v}")
        else:
            # If the value is not a key, it is a direct value which is input in the subtree,
            # for example `<SubTree ID="subtree1" in="500" />`.
            v = resolve_direct_value_remapping(v, remapping_table)
            logger.debug(f"[Direct value] Adding remapping from parent remapping table: {k} -> {v}")
        new_remapping[k] = v
    logger.debug(f"Subtree '{elem.attrib['ID']}' new remapping table: {new_remapping}")
    return new_remapping


def build_port_remappings(
    elem: ET.Element,
    class_: type[PortsMixin],
    remapping_table: dict[str, str],
    subtree_namespace: str,
    logger: PortsLogger = NOOP_LOGGER,
) -> dict[str, str]:
    """
    Build ``{port_name -> absolute_key}`` for any :class:`PortsMixin` node from XML attributes.

    Mirrors the logic used for :class:`PortsMixin` leaves:

    - Attributes must correspond to declared input/output ports
      (otherwise ``NotImplementedError``).
    - Each attribute value may be a ``"{key}"`` reference or a direct value; both are resolved
      to absolute keys via the remapping table (adding entries as needed).
    - If the natural in-namespace key (``/{ns}/{port}``) differs from the resolved absolute key,
      a remapping is recorded.
    """
    # Build port remappings for this PortsMixin node.
    # This is a dictionary that maps port names to their absolute keys.
    port_remappings = {}  # {port: remap_to_absolute_key}
    for attrib_key, attrib_value in elem.attrib.items():
        if attrib_key == "name":
            continue
        if attrib_key not in class_.input_ports() and attrib_key not in class_.output_ports():
            logger.debug(
                f"Attribute '{attrib_key}' is not defined in class '{class_.__name__}'. "
                f"Treating as additional parameter."
            )
            continue

        if is_key(attrib_value):
            # If the value is a key, and we haven't yet encountered it, we need to add it to the remapping table,
            # so that it can be resolved to its absolute path.
            add_new_key_to_remapping_table(
                attrib_value,
                remapping_table=remapping_table,
                subtree_namespace=subtree_namespace,
            )
            # Resolve the value to its absolute path.
            absolute_key = resolve_key_remapping(attrib_value, remapping_table)
        else:
            # If the value is not a key, it is a direct value which is input in the node.
            # We need to add a new blackboard key in this namespace.
            # Then we need to add the key to the remapping table.
            add_new_key_to_remapping_table(
                attrib_value,
                remapping_table=remapping_table,
                subtree_namespace=subtree_namespace,
            )
            absolute_key = resolve_direct_value_remapping(attrib_value, remapping_table)

        # Always record the remapping if an explicit attribute is provided. Even if the
        # resolved key matches the natural namespace location, the explicit declaration
        # means the user intends to share that port, and the PortsMixin will use the
        # provided absolute key instead of the node-scoped default.
        port_remappings[attrib_key] = absolute_key

    return port_remappings


def instantiate_ports_node(
    elem: ET.Element,
    cls: type[PortsMixin],
    node_registry: dict,
    remapping_table: dict[str, str],
    subtree_namespace: str,
    logger: PortsLogger = NOOP_LOGGER,
    constructor_kwargs: dict | None = None,
    parent_names_str: str = "",
) -> PortsMixin:
    """
    Instantiate any PortsMixin-based node (leaf or composite).

    - looks up the class via `node_registry` (validated with `get_class_from_registry`)
    - builds port remappings from `elem.attrib`
    - constructs the instance (using `name` attribute or class_name)
    - calls `setup_ports(...)`

    Args:
        elem: The XML element to parse.
        cls (type[py_trees.behavious.Behaviour]): A behaviour class type.
        node_registry (dict): Mapping from class names (str) to callables (constructors or partials)
            that return ``PortsMixin``-derived instances.
        remapping_table (dict): Mapping from keys (str) to absolute keys (str).
        subtree_namespace (str): The namespace for this subtree.
        logger: Optional logger-like object.
        constructor_kwargs (dict | None): Additional keyword arguments to pass to the constructor.
        parent_names_str (str): Dot-separated string of parent names for logging context and generating node names.

    Returns:
        PortsMixin: the fully initialised node.
    """
    port_remappings = build_port_remappings(
        elem=elem,
        class_=cls,
        remapping_table=remapping_table,
        subtree_namespace=subtree_namespace,
        logger=logger,
    )

    instance_name = generate_node_name(
        explicit_name=elem.attrib.get("name", None),
        general_name=elem.tag,
        prefix=parent_names_str,
    )
    logger.debug(f"PortsMixin node '{instance_name}' in namespace {subtree_namespace} remappings: {port_remappings}")

    constructor_kwargs = constructor_kwargs or {}  # create constructor_kwargs
    for attrib_key, attrib_value in elem.attrib.items():
        if attrib_key == "name":
            continue
        if attrib_key not in cls.input_ports() and attrib_key not in cls.output_ports():
            if is_key(attrib_value):
                raise ValueError(
                    f"'{elem.tag}'(name='{instance_name}'): Port remappings are "
                    f"not supported for non-port attributes ('{attrib_key}'='{attrib_value}'). "
                )
            # Consistency check: if constructor_kwargs already has this key, and it's conflicting,
            # print a warning.
            if constructor_kwargs and attrib_key in constructor_kwargs:
                existing_entry = constructor_kwargs[attrib_key]
                if existing_entry != attrib_value:
                    logger.warning(
                        f"Conflicting values for attribute '{attrib_key}': "
                        f"{existing_entry} (existing) vs {attrib_value} (new). "
                        f"Using {attrib_value}."
                    )
            logger.debug(
                f"Attribute '{attrib_key}' is not defined in class '{cls.__name__}'. Treating as additional parameter."
            )
            constructor_kwargs[attrib_key] = attrib_value

    ctor_callable = node_registry[elem.tag]
    # Try to convert the constructor arguments to the correct type.
    ignore_keys = {"child", "children", "behaviour_class_name"}
    constructor_kwargs, success = apply_type_hints(ctor_callable, constructor_kwargs, logger=logger, ignore=ignore_keys)
    if not success:
        logger.warning(
            "Failed to apply type hints to constructor arguments. See error log. Proceeding, but leaving "
            "the conversion to the constructors."
        )
    try:
        # Pass the behaviour_class_name (the tag/registry name) to the constructor
        node: PortsMixin = ctor_callable(
            name=instance_name,
            behaviour_class_name=elem.tag,
            **constructor_kwargs,
        )
    except Exception as e:  # Catch everything that may go wrong in the constructor
        raise XMLParserError(
            f"Failed to instantiate '{elem.tag}' with args {constructor_kwargs} "
            f"(check: does the instantiated class have kwargs in __init__?): {e}"
        ) from e

    node.setup_ports(
        port_remappings=port_remappings,
        subtree_namespace=subtree_namespace,
        logger=logger,
    )
    return node


def build_tree_from_xml(
    elem: ET.Element,
    remapping_table: dict[str, str],
    node_registry: dict,
    bt_index: dict,
    logger: PortsLogger = NOOP_LOGGER,
    subtree_namespace: str = "/",
    parent_names_str: str = "",
) -> py_trees.behaviour.Behaviour:
    """
    Recursively build the tree from XML element.

    Args:
        elem: XML element
        remapping_table (dict[str, str]): Remapping table.
        node_registry (dict): Mapping from class names (str) to callables (constructors or partials)
            that return ``PortsMixin``-derived instances.
        bt_index (dict[str, BehaviorTree]): dictionary {ID: BehaviorTree element} for subtree lookup.
        subtree_namespace (str): current blackboard namespace.
        logger: Optional logger-like object.
        parent_names_str (str): Dot-separated string of parent names for logging context and generating node names.

    Returns:
        py_trees.behaviour.Behaviour

    Raises:
        NotImplementedError: For unknown composite tags or unsupported ports.
        ValueError: For missing trees or unsupported tags.
        AssertionError: If a <BehaviorTree> does not have exactly one child.
    """
    tag = elem.tag.lower()
    logger.debug(f"Processing tag: '{elem.tag}' with attributes {elem.attrib}. Remapping table: {remapping_table}")
    if elem.tag in node_registry:
        # Prioritize nodes that are in the registry, which could even be user overrides for built-ins.
        # This is any PortsMixin node (any PortsMixin + Behaviour combination).
        try:
            cls = get_class_from_registry(elem.tag, node_registry)
        except KeyError as e:
            raise NotImplementedError(
                f"Class name '{tag}' not found in node_registry. Supporting other types is still TODO."
            ) from e

        if not (issubclass(cls, PortsMixin) and issubclass(cls, py_trees.behaviour.Behaviour)):
            raise TypeError(
                f"XML tag '{elem.tag}' did not instantiate a PortsMixin + "
                f"py_trees.behaviour.Behaviour; got {cls.__name__}"
            )

        children = []
        constructor_kwargs: dict | None = None
        is_decorator = issubclass(cls, py_trees.decorators.Decorator)
        is_composite = issubclass(cls, py_trees.composites.Composite)
        if is_decorator and len(elem) != 1:
            raise XMLParserError(f"Decorator node '{tag}' must have exactly 1 child, found {len(elem)}.")
        if is_decorator or is_composite:
            for child in elem:
                # Use no general name fallback for the parent_names_str to pass to the children.
                # Otherwise, the generated node names will get too long and not very readable.
                # It also means that we lose the guarantee of unique names, but that can be avoided by
                # assigning an explicit name to the parent tags.
                concise_parent_names_str = generate_node_name(
                    explicit_name=elem.attrib.get("name", None),
                    general_name="",
                    prefix=parent_names_str,
                    no_uuid=True,
                )
                child_node = build_tree_from_xml(
                    child,
                    remapping_table,
                    node_registry,
                    bt_index,
                    logger=logger,
                    subtree_namespace=subtree_namespace,
                    parent_names_str=concise_parent_names_str,
                )
                if not isinstance(child_node, py_trees.behaviour.Behaviour):
                    raise TypeError(f"Child node of type {type(child_node).__name__} is not a valid py_trees Behavior.")
                children.append(child_node)

        if is_decorator:
            constructor_kwargs = {"child": children[0]}  # already checked its length
        elif is_composite:
            constructor_kwargs = {"children": children}

        logger.debug(f"Creating PortsMixin node for tag {elem.tag}.")
        node = instantiate_ports_node(
            elem=elem,
            cls=cls,
            node_registry=node_registry,
            remapping_table=remapping_table,
            subtree_namespace=subtree_namespace,
            logger=logger,
            constructor_kwargs=constructor_kwargs,
            parent_names_str=parent_names_str,
        )
        return node

    elif tag in PARENT_NODES_TAGS:
        # Composite/Decorator nodes which can have children.
        # TODO: Everything in this section should be eventually replaced with actual
        # built-in behaviours with ports that are part of the node registry.
        node_name = generate_node_name(
            explicit_name=elem.attrib.get("name", None),
            general_name=elem.tag,
            prefix=parent_names_str,
        )
        logger.debug(
            f"Entering composite node: {tag} (given name {node_name} with parent names {parent_names_str}). "
            "Build children first, so we can pass them to the constructor."
        )
        children = []
        for child in elem:
            # Use no general name fallback for the parent_names_str to pass to the children.
            # Otherwise, the generated node names will get too long and not very readable.
            # It also means that we lose the guarantee of unique names, but that can be avoided by
            # assigning an explicit name to the parent tags.
            concise_parent_names_str = generate_node_name(
                explicit_name=elem.attrib.get("name", None),
                general_name="",
                prefix=parent_names_str,
                no_uuid=True,
            )
            child_node = build_tree_from_xml(
                child,
                remapping_table,
                node_registry,
                bt_index,
                logger=logger,
                subtree_namespace=subtree_namespace,
                parent_names_str=concise_parent_names_str,
            )
            if not isinstance(child_node, py_trees.behaviour.Behaviour):
                raise TypeError(f"Child node of type {type(child_node).__name__} is not a valid py_trees Behavior.")
            children.append(child_node)

        memory = elem.attrib.get("memory", "true").lower() == "true"
        node: py_trees.behaviour.Behaviour
        if tag == "sequence":
            node = py_trees.composites.Sequence(name=node_name, memory=memory, children=children)
        elif tag == "selector" or tag == "fallback":
            node = py_trees.composites.Selector(name=node_name, memory=memory, children=children)
        elif tag == "parallel":
            policy = elem.attrib.get("policy", "success_on_one")
            mapping = {
                "success_on_all": py_trees.common.ParallelPolicy.SuccessOnAll,
                "success_on_one": py_trees.common.ParallelPolicy.SuccessOnOne,
                "success_on_selected": py_trees.common.ParallelPolicy.SuccessOnSelected,
            }
            node = py_trees.composites.Parallel(
                name=node_name,
                policy=mapping.get(policy, py_trees.common.ParallelPolicy.SuccessOnOne)(),  # type: ignore
                children=children,
            )
        elif tag in DECORATOR_NODES:
            # Instantiate built-in decorators, including extracting constructor arguments using type hints.
            constructor_kwargs: dict[str, Any] = {}
            cls = DECORATOR_NODES[tag]
            if len(children) != 1:
                raise ValueError(f"Decorator '{elem.tag}' must have exactly one child, but got {len(children)}.")
            constructor_kwargs["child"] = children[0] if children else None

            for key in elem.keys():
                if key == "name":
                    continue
                constructor_kwargs[key] = elem.attrib.get(key)

            ignore_keys = {"child", "children", "behaviour_class_name"}
            constructor_kwargs, success = apply_type_hints(cls, constructor_kwargs, logger=logger, ignore=ignore_keys)
            if not success:
                logger.warning(
                    "Failed to apply type hints to constructor arguments. See error log. "
                    "Proceeding, but leaving the conversion to the constructors."
                )

            try:
                node = cls(name=node_name, **constructor_kwargs)
            except Exception as e:  # Catch everything that may go wrong in the constructor
                raise XMLParserError(
                    f"Failed to instantiate '{tag}' with args {constructor_kwargs} "
                    f"(check: does the instantiated class have kwargs in __init__?): {e}"
                ) from e

        else:
            raise NotImplementedError(f"Unknown composite tag: {tag}")

        if not isinstance(node, py_trees.composites.Composite) and not isinstance(node, py_trees.decorators.Decorator):
            raise TypeError(f"XML tag '{elem.tag}' did not instantiate a Composite; got {type(node).__name__}")
        return node
    elif tag == "behaviortree":
        # This branch is reached when the parser encounters a <BehaviorTree> tag during recursion.
        # There are two main cases:
        #   1. At the top level, the parser starts with the <BehaviorTree> whose ID matches main_tree_to_execute.
        #   2. When recursing into a subtree, the parser finds the referenced <BehaviorTree> by ID and enters here.
        # In both cases, the <BehaviorTree> tag is a container for the actual tree structure (usually a Sequence,
        # Selector, etc.).
        # The parser expects the <BehaviorTree> to have a single child (the root control node of the tree).
        # This is because the subtree is to be used like a whole XML, which has one root node.
        # It recurses into that child, passing along the current remapping table and other context.
        # This is NOT a subtree instantiation (which is handled by the 'subtreeplus'/'subtree' branch),
        # but simply the entry point for parsing the structure of a tree or subtree definition.
        logger.debug(f"Entering <BehaviorTree> ID='{elem.attrib.get('ID', '')}'")
        child_elems = list(elem)
        assert len(child_elems) == 1, (
            f"<BehaviorTree ID='{elem.attrib.get('ID', '')}'> must have exactly one child (the root node), "
            f"but found {len(child_elems)} children."
        )
        return build_tree_from_xml(
            child_elems[0],
            remapping_table,
            node_registry,
            bt_index,
            logger=logger,
            subtree_namespace=subtree_namespace,
            parent_names_str=parent_names_str,
        )
    elif tag in ("subtreeplus", "subtree"):
        # This branch is reached when the parser encounters a <subtreeplus> or <subtree> tag during recursion.
        # These tags are used to instantiate subtrees.
        # It recurses into that child, passing along the current remapping table and the subtree namespace.
        # The remapping table is updated to include the remappings from the <subtreeplus> or <subtree> element.
        # The subtree is then instantiated with the new remapping table.
        logger.debug(f"Instantiating subtree '{elem.attrib['ID']}' with remapping table BEFORE: {remapping_table}")
        subtree_id = elem.attrib["ID"]
        subtree_name = elem.attrib.get("name", str(uuid.uuid4()))
        if subtree_id not in bt_index:
            raise ValueError(f"Subtree ID '{subtree_id}' not found in XML.")
        subtree_elem = bt_index[subtree_id]
        # Build the new namespace by prepending the subtree ID
        new_namespace = get_absolute_reference(subtree_name, subtree_namespace)
        # Build the new remapping table for this subtree.
        new_remapping = build_subtree_remapping(elem, remapping_table, subtree_namespace, logger)
        # Recursively build the subtree with the new remapping table
        return build_tree_from_xml(
            subtree_elem,
            new_remapping,
            node_registry,
            bt_index,
            logger=logger,
            subtree_namespace=new_namespace,
            parent_names_str=((parent_names_str + ".") if parent_names_str else "") + subtree_name,
        )
    else:
        logger.error(f"Unsupported tag encountered: {elem.tag}")
        raise ValueError(
            f"Unsupported tag '{elem.tag}' encountered in XML. "
            "This is not a known composite, subtree, or PortsMixin node."
        )


def _collect_bt_ids(root: ET.Element) -> set[str]:
    """
    Collect IDs of all <BehaviorTree> elements in an XML root.

    Args:
        root: XML element that contains zero or more <BehaviorTree> children.

    Returns:
        A set of all BehaviorTree IDs found under 'root'.

    Raises:
        KeyError: If any <BehaviorTree> is missing the 'ID' attribute.
    """
    return {bt.attrib["ID"] for bt in root.findall("BehaviorTree")}


def _resolve_import_path(src: str, base_dir: str, search_paths: list[str] | None) -> str:
    """
    Resolve import path to an absolute file on disk.

    Resolution order:
        1) If 'src' is absolute and exists, use it.
        2) Relative to 'base_dir' (the directory of the including XML).
        3) Each directory in 'search_paths' (if provided).

    Args:
        src: Path from the <Import src="..."> or <Include file="..."> attribute.
        base_dir: Directory of the XML that contains the import.
        search_paths: Optional additional directories to search.

    Returns:
        Absolute, existing path to the imported XML.

    Raises:
        FileNotFoundError: If no candidate path exists.
    """
    candidates = []
    if os.path.isabs(src):
        candidates.append(src)
    candidates.append(os.path.join(base_dir, src))
    if search_paths:
        candidates.extend(os.path.join(p, src) for p in search_paths)

    for c in candidates:
        if os.path.exists(c):
            return os.path.abspath(c)

    raise FileNotFoundError(f"Import not found for '{src}'. Tried: {candidates}")


def _inline_imports_into_root(
    root: ET.Element,
    current_file: str,
    logger: PortsLogger,
    search_paths: list[str] | None,
    visited: set[str] | None = None,
) -> None:
    """
    Inline top-level <Import/> / <Include/> directives into 'root' in-place.

    This performs a pre-processing step so that the rest of the parser can
    remain unchanged. It:
      - Resolves each import path.
      - Prevents import cycles.
      - Parses the imported XML and recursively inlines imports there as well.
      - Appends *all* <BehaviorTree> elements from the imported XML to 'root'.
      - Raises if any imported BehaviorTree ID already exists in 'root'.
      - Removes the <Import/> / <Include/> element after inlining.

    Notes:
      - Only top-level imports (direct children of 'root') are supported.
      - No renaming/prefixing: ID collisions are treated as errors.

    Args:
        root: The root element of the currently loaded XML.
        current_file: Absolute path of the XML file backing 'root'.
        logger: Logger-like object or None for debug messages.
        search_paths: Optional extra directories to search for import files.
        visited: Set of absolute file paths already processed (cycle guard).

    Raises:
        ValueError: On BehaviorTree ID collisions or malformed imports.
        FileNotFoundError: If an import target cannot be found.
    """
    if visited is None:
        visited = set()

    base_dir = os.path.dirname(os.path.abspath(current_file))
    try:
        existing_ids = _collect_bt_ids(root)
    except KeyError as e:
        raise ValueError(f"BehaviorTree missing ID in '{current_file}'") from e

    # Only handle top-level children named Import/Include (case-insensitive).
    imports = [child for child in list(root) if child.tag.lower() in ("import", "include")]

    logger.debug(f"Found {len(imports)} import(s) in '{current_file}'")

    for imp in imports:
        logger.debug(f"Processing import directive '{imp.tag}': {imp.attrib}")
        # Support either src= or file=
        src = imp.attrib.get("src") or imp.attrib.get("file")
        if not src:
            raise ValueError(f"<{imp.tag}> requires 'src' or 'file' attribute")

        # Resolve path relative to the including file (or search_paths)
        resolved = _resolve_import_path(src, base_dir, search_paths)

        # Cycle detection
        if resolved in visited:
            # Don't re-add the same file
            continue

        visited.add(resolved)

        logger.debug(f"Inlining {imp.tag} from '{resolved}'")

        # Parse the imported file and inline its imports first (depth-first)
        imported_tree = ET.parse(resolved)
        imported_root = imported_tree.getroot()
        _inline_imports_into_root(imported_root, resolved, logger, search_paths, visited)
        logger.debug(f"Imported XML: '{ET.tostring(imported_root, encoding='unicode')}'")

        # Append all BehaviorTrees from imported file, but forbid ID collisions
        for bt in imported_root.findall("BehaviorTree"):
            logger.debug(f"Processing ID={bt.attrib.get('ID')}")
            bt_id = bt.attrib.get("ID")
            if not bt_id:
                raise ValueError(f"Imported BehaviorTree missing ID in '{resolved}'")

            if bt_id in existing_ids:
                # Simple mode: collisions are hard errors
                raise ValueError(f"BehaviorTree ID collision: '{bt_id}' already exists while importing '{resolved}'")

            # Deep-copy to detach from the imported tree and append into 'root'
            root.append(deepcopy(bt))
            existing_ids.add(bt_id)
            logger.debug(f"Imported BehaviorTree ID='{bt_id}' from '{resolved}'")

        # Remove the import directive after successful inlining
        root.remove(imp)

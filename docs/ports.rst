.. _ports-section-label:

Ports (Experimental)
====================

.. note::

   The ports module (:mod:`py_trees.ports`) and the XML parser
   (:mod:`py_trees.parsers.behaviour_tree_xml`) are **experimental**.
   Their API may change between releases.

Overview
--------

**Ports** add a structured way of wiring up data exchange between nodes.
Each node defines the data it reads and writes as **input** and **output** ports.
The ports are wired to blackboard keys (the *remapping*), and port values are type-checked at runtime.
This allows for a defined and constrained way of data exchange between nodes which is less error-prone and easier to debug than simply writing data on the blackboard and letting nodes read from and write to that entry directly.

Why use ports?
~~~~~~~~~~~~~~

Using ports instead of ad-hoc blackboard reads and writes pays off in several concrete ways:

* **Explicit data contracts.**
  A node's ``INPUT_PORTS`` and ``OUTPUT_PORTS`` declarations *are* its data-flow API.
  A reader can see at a glance what a node consumes and produces without reading through its :meth:`~py_trees.behaviour.Behaviour.update` method.

* **Structured, early error detection.**
  Port values are type-checked at runtime (``TypeError`` on mismatched writes/reads), required inputs without data raise :class:`~py_trees.ports.NoDataAvailable` instead of returning ``None`` silently, and misconfiguration (a remap for a port that isn't declared, or a port type that can't accept the wiring) surfaces at setup time rather than deep in a tick.

* **XML authoring.**
  Once a library of port-enabled nodes exists, trees become declarative data.
  Non-programmers can read and edit tree structure (and the data wiring) without touching Python.
  See the :ref:`XML parser section <ports-xml-parser-label>` below.

* **Reusable subtrees via rewiring.**
  A subtree is configured from the outside by rewiring its port remappings.
  A subtree can be re-used with different inputs and outputs without touching the internal code.
  In combination with the XML parser, this concept of re-usable subtrees becomes really powerful to quickly put together new behaviors.

* **Automatic subtree isolation.**
  Sibling subtrees can freely reuse the same port names internally; the subtree namespace scopes them on the blackboard so they don't collide.

* **Refactoring safety.**
  Changing the blackboard key a data item lives under is a *rewiring* change at setup time, not a code change scattered across every node that used to read or write that key by name.

* **Easier to test.**
  A single node can be exercised in isolation by wiring its ports to known blackboard keys, seeding the inputs, and ticking once.
  No tree scaffolding required; the ports contract becomes the test surface.

The primary API is :class:`py_trees.ports.PortsMixin`.
Concrete nodes typically inherit from the convenience base
:class:`py_trees.ports.BehaviourWithPorts`, which combines the mixin with
:class:`py_trees.behaviour.Behaviour`:

.. code-block:: python

   import py_trees
   from py_trees.ports import BehaviourWithPorts, PortInformation

   class Multiply(BehaviourWithPorts):
       INPUT_PORTS = {
           "a": PortInformation(data_type=float, required=True),
           "b": PortInformation(data_type=float, required=True),
       }
       OUTPUT_PORTS = {
           "product": PortInformation(data_type=float, required=True),
       }

       def update(self):
           self._set_output("product", self.get_input("a") * self.get_input("b"))
           return py_trees.common.Status.SUCCESS

Wiring (remapping) and type checking
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Ports become usable after :meth:`~py_trees.ports.PortsMixin.setup_ports` has been called with the *port remappings* (the "wiring").
Remappings map each port to an absolute or relative blackboard key.
The purpose of remapping is to "wire" one node's output port(s) to another node's input port(s) so they can exchange data.

.. code-block:: python

   node = Multiply(name="mul")
   node.setup_ports(
       port_remappings={
           "a":       "/numbers/a",
           "b":       "/numbers/b",
           "product": "/numbers/product",
       }
   )

In the example above, another node's output ports would typically be remapped to ``/numbers/a`` and ``/numbers/b`` and thereby provide the input for the ``Multiply`` node.

.. note:: Why is ``setup_ports()`` a separate call?
    Because the remapping table usually cannot be computed until the entire tree topology is known — either the user assembles it by hand or a parser generates it from e.g. XML (more on that next).
    See :class:`py_trees.ports.PortsMixin` for the full contract and semantics.

Default values
~~~~~~~~~~~~~~

A port can declare a ``default_value`` next to its type, so the fallback lives with the port declaration rather than being repeated at every call site:

.. code-block:: python

   INPUT_PORTS = {
       "timeout": PortInformation(data_type=float, default_value=5.0),
   }

   def update(self):
       timeout = self.get_input("timeout")   # 5.0 unless something wrote to the port

The semantics are:

* **Validated at declaration time.**
  The default is type-checked against ``data_type`` when the :class:`~py_trees.ports.PortInformation` is constructed, so a mismatch raises ``TypeError`` where the port is declared instead of surfacing on a tick.

* **Applied whenever no data is available.**
  That covers both an unwired port and a port wired to a key nothing has written to yet.
  Any actual data on the port wins over the default.

* **They make a port satisfiable, including a required one.**
  ``required=True`` together with a default reads as "this node always needs a value here, and here is the fallback"; such a port never raises :class:`~py_trees.ports.NoDataAvailable`.
  It is also not registered as a *required* blackboard key, so :meth:`~py_trees.blackboard.Client.verify_required_keys_exist` does not trip over it.

* **A call-site default still wins.**
  ``get_input("timeout", default=1.0)`` overrides the declared default for that read.

* **Input defaults are read-side only.**
  They are never written to the blackboard, so wiring a defaulted input port to a shared key does not seed that key for other readers.
  Container defaults (e.g., a list) are deep-copied on the way out, so a caller mutating the returned value cannot corrupt the declaration shared by every instance of the class.

* **Output defaults are seeded onto the blackboard.**
  An output port's default is written by ``setup_ports()``, so nodes wired to that port read a value before the producing node has ticked; whatever the node writes later replaces it.
  Unlike input defaults, this does touch shared state — if two output ports are wired to the same key, the last one set up wins.

* **Reset returns a port to its default.**
  :meth:`~py_trees.ports.PortsMixin.reset_port` (and ``reset_all_output_ports()``) re-seeds an output port that declares a default instead of leaving it empty, so the guarantee that readers always see a value survives a "new data epoch".
  Ports without a default are cleared as before.

Note that ``default_value=None`` means "no default declared" — ``None`` is not supported as a port value in any case.

.. _ports-xml-parser-label:

Experimental XML parser
-----------------------

The module :mod:`py_trees.parsers.behaviour_tree_xml` ships an (experimental) parser for the `BehaviorTree.CPP <https://www.behaviortree.dev/docs/learn-the-basics/main_concepts>`_ XML format.
It builds a py_trees tree from an XML file and auto-generates the port remappings for every node.

.. code-block:: python

   from py_trees.parsers.behaviour_tree_xml import parse_behaviour_tree_xml

   root = parse_behaviour_tree_xml("my_tree.xml")

How the parser maps each XML tag to a Python class is covered in
:ref:`Resolving node classes <ports-xml-class-resolution-label>` just below.
The :ref:`demos <ports-demos-section-label>` further down show complete working examples.

.. _ports-xml-class-resolution-label:

Resolving node classes
~~~~~~~~~~~~~~~~~~~~~~~~

An XML tag like ``<MyNode>`` is just a name --- before the parser can build the node it has
to find the Python class that name refers to. This section explains how that lookup happens
and how you can customize it.

There is a ``node_registry`` argument which controls that lookup, and takes one of two forms.

**1. Auto (the default).** Every concrete :class:`~py_trees.ports.PortsMixin` subclass
registers itself (under its class name) the moment it is defined. So as long as your node
classes have been imported, the parser resolves every tag with no extra configuration ---
``node_registry="auto"`` is the default, so you pass nothing:

.. code-block:: python

   from py_trees.parsers.behaviour_tree_xml import parse_behaviour_tree_xml
   from my_nodes import MyNode, OtherNode  # importing is enough to register them

   root = parse_behaviour_tree_xml("my_tree.xml")

**2. An explicit dict.** Pass a ``{tag: class-or-callable}`` mapping to use *exactly* that
mapping and ignore the auto-registry. This is the form to reach for when you need to inject
a dependency, alias a tag, or sandbox the parser to a fixed set of classes:

.. code-block:: python

   root = parse_behaviour_tree_xml(
       "my_tree.xml",
       node_registry={"MyNode": MyNode, "OtherNode": OtherNode},  # only these, no auto
   )

To **combine** auto-registration with a few explicit entries --- most commonly to inject a
runtime dependency via ``functools.partial`` (a partial isn't a class, so it can't live in
the auto-registry) --- spread the auto-registry into your dict and override the entries you
need:

.. code-block:: python

   import py_trees
   from functools import partial

   root = parse_behaviour_tree_xml(
       "my_tree.xml",
       node_registry={
           **py_trees.ports.get_ports_registry(),       # everything auto-registered, plus...
           "Wait": partial(Wait, factory=my_factory),   # ...a dependency-injected override
       },
   )

**Aliasing.** To expose a class under an extra tag, register it under that tag --- either
inline on the class definition or via the helper for classes you cannot modify. Either way it
then resolves under the alias in the ``"auto"`` path:

.. code-block:: python

   class Foo(PortsMixin, py_trees.behaviour.Behaviour, tag="Bar"):
       ...

   # or, for a third-party class:
   from py_trees.ports import register_ports_class
   register_ports_class("Bar", Foo)

Pass ``register=False`` on a class definition to keep a particular subclass out of the
registry. Still-abstract classes (e.g. :class:`~py_trees.ports.BehaviourWithPorts` itself,
which does not declare ``INPUT_PORTS`` / ``OUTPUT_PORTS``) are never registered.

.. _ports-xml-attributes-label:

XML attributes: ports *and* constructor arguments
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Attributes on a node's XML tag serve **two** distinct purposes:

1. Attribute names that match a declared port (``INPUT_PORTS`` or ``OUTPUT_PORTS``) are treated as **port remappings**.
   Values may be ``{curly_key}`` references (wired to the remapping table) or literal constants (type-converted according to the port's declared type).

2. Attribute names that do **not** match any declared port are treated as **constructor keyword arguments** and forwarded to the class constructor.
   Values are type-converted based on the constructor's type annotations (strings to ``int`` / ``float`` / ``bool`` / enum / etc.); un-annotated parameters receive the raw string.
   ``{curly_key}`` references are *not* allowed for constructor kwargs --- they raise a ``ValueError`` at parse time, because constructor kwargs can't be re-wired at runtime the way ports can.

Example::

   class Greeting(BehaviourWithPorts):

       INPUT_PORTS = {"name_key": PortInformation(data_type=str, required=True)}
       OUTPUT_PORTS = {"greeting": PortInformation(data_type=str, required=True)}

       def __init__(self, name: str, prefix: str = "Hello", **kwargs):
           super().__init__(name=name, **kwargs)
           self.prefix = prefix        # not a port; populated from XML attribute

       def update(self):
           self._set_output("greeting", f"{self.prefix}, {self.get_input('name_key')}!")
           return py_trees.common.Status.SUCCESS

.. code-block:: xml

   <Greeting name="hello_node" name_key="{target}" prefix="Howdy"/>
   <!--       ^^^ behaviour name   ^^^ port remap    ^^^ ctor kwarg  -->

.. _ports-xml-default-inputs-label:

Setting default inputs for subtrees
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

A ``<BehaviorTree>`` definition can carry attributes of its own.
Every attribute other than ``ID`` and ``name`` is a **default input** for that tree:
a value used for the matching key whenever an instance of the tree does not supply one itself.
This lets a reusable subtree ship with sensible values baked in, so callers only have to override what they actually care about.

.. code-block:: xml

   <root main_tree_to_execute="MainTree">
     <BehaviorTree ID="Greeter" greeting="Hello" volume="5">
       <Sequence>
         <Speak name="Speak" text="{greeting}" volume="{volume}"/>
       </Sequence>
     </BehaviorTree>

     <BehaviorTree ID="MainTree">
       <Sequence>
         <SubTree ID="Greeter" name="Default"/>                        <!-- "Hello", 5 -->
         <SubTree ID="Greeter" name="Loud" volume="11"/>               <!-- "Hello", 11 -->
         <SubTree ID="Greeter" name="Custom" greeting="Howdy"/>        <!-- "Howdy", 5  -->
       </Sequence>
     </BehaviorTree>
   </root>

Resolution is per key: an attribute on the ``<SubTree>`` element wins, and every key it leaves out
falls back to the default on the ``<BehaviorTree>`` definition.
Defaults are written exactly like any other XML attribute value.
A literal constant is type-converted according to the declared port type
(so ``volume="5"`` reaches an ``int`` port as ``5``), and a ``{curly_key}`` reference is wired through the remapping table as usual.
Note that a ``{curly_key}`` default is resolved in the scope of whoever *instantiates* the subtree,
not inside the subtree itself, so it behaves like a remapping the caller would have written by hand.
Literal defaults are usually the clearer choice.

Defaults for the top-level tree
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The same mechanism applies to the tree named by ``main_tree_to_execute``.
Attributes on *its* ``<BehaviorTree>`` element are the defaults for the tree as a whole.
Since there is no enclosing ``<SubTree>`` element to override them from, those defaults are overridden
from Python instead, via the ``input_mappings`` argument of
:func:`~py_trees.parsers.behaviour_tree_xml.parse_behaviour_tree_xml`:

.. code-block:: xml

   <root main_tree_to_execute="MainTree">
     <BehaviorTree ID="MainTree" greeting="Hello">
       <Sequence>
         <Speak name="Speak" text="{greeting}" volume="{volume}"/>
       </Sequence>
     </BehaviorTree>
   </root>

.. code-block:: python

   root = parse_behaviour_tree_xml(
       "my_tree.xml",
       input_mappings={"greeting": "Howdy", "volume": "11"},
   )

This makes the top-level tree parameterisable from the calling application without having to
edit (or template) the XML for each run.

A few rules to be aware of:

* ``input_mappings`` values must be **strings**, matching how they would have been written as
  XML attributes (pass ``"11"``, not ``11``). A non-string value raises ``ValueError`` at parse time.
* ``input_mappings`` both overrides existing defaults and supplies keys the XML never defaulted at all (``volume`` in the example above).
* A required input port left with neither a default nor a mapping is not an error at parse time.
  It raises :class:`~py_trees.ports.NoDataAvailable` if and when the node first reads it during a tick.

.. _ports-xml-includes-label:

Composing trees from multiple files
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Subtrees do not have to live in the same file as the tree that uses them.
A top-level ``<Include>`` (or ``<Import>``) element pulls all ``<BehaviorTree>``
definitions from another XML file into the current document before parsing,
so you can maintain a library of reusable subtrees in separate files:

.. code-block:: xml

   <!-- subtree_library.xml -->
   <root>
     <BehaviorTree ID="LibraryTree">
       <Sequence>
         <Producer name="LibProducer" output="{lib_out}"/>
       </Sequence>
     </BehaviorTree>
   </root>

.. code-block:: xml

   <!-- main_tree.xml -->
   <root main_tree_to_execute="MainTree">
     <Include src="subtree_library.xml"/>
     <BehaviorTree ID="MainTree">
       <Sequence>
         <SubTree ID="LibraryTree" name="Library" lib_out="{final}"/>
         <Consumer name="FinalConsumer" input="{final}"/>
       </Sequence>
     </BehaviorTree>
   </root>

The path in ``src=`` (``file=`` is accepted as an alias) is resolved in order:

1. As an absolute path, if it is one.
2. Relative to the directory of the *including* XML file.
3. Against each directory in the ``search_paths`` argument of
   :func:`~py_trees.parsers.behaviour_tree_xml.parse_behaviour_tree_xml`.

So if the library file lives somewhere other than next to the main file --- say a
shared ``subtrees/`` directory --- pass that directory via ``search_paths``:

.. code-block:: python

   root = parse_behaviour_tree_xml(
       "main_tree.xml",
       search_paths=["/path/to/subtrees"],
   )

A few more rules to be aware of:

* Includes are only recognised as **top-level** children of ``<root>``; an
  ``<Include>`` nested inside a ``<BehaviorTree>`` is an error.
* Includes are processed **recursively** --- an included file may itself include
  further files --- and cycles are detected and skipped.
* All BehaviorTree IDs must be unique across the main file and everything it
  includes; a collision raises ``ValueError``.
* ``main_tree_to_execute`` may refer to a tree defined in an included file.

Scope, limitations, and how to extend
-------------------------------------

The current ports framework is deliberately minimal.
It ships the :class:`~py_trees.ports.PortsMixin` contract, the convenience :class:`~py_trees.ports.BehaviourWithPorts` base, an experimental XML parser, and four demos.
It is the base for extensions to be added in future.
A few things you should be aware of, and suggestions on how to fill the gaps yourself:

**1. No port-aware behaviours, decorators, or composites are shipped.**

   ``py_trees.ports`` provides the *mechanism* for typed input/output ports.
   It does **not** currently ship any concrete behaviours that use ports (e.g. there is no ``Retry`` with ports).
   The library of port-enabled nodes is the user's domain: you define ``PortsMixin``-derived classes that actually *do something* with the input/output data.

**2. Built-in decorators and composites cannot be wired through ports from XML.**

   The XML parser supports four built-in composite tags natively (``<Sequence>``, ``<Selector>`` / ``<Fallback>``, ``<Parallel>``).
   For these tags, only a **fixed** set of XML attributes is consumed:

   * ``<Sequence>`` / ``<Selector>`` / ``<Fallback>``: ``name``, ``memory``
   * ``<Parallel>``: ``name``, ``policy`` (one of ``success_on_all``,
     ``success_on_one``, ``success_on_selected``)

   **Any other attribute on a built-in composite tag is silently ignored.**
   For example, ``<Parallel synchronise="true">`` has no effect, and port-style attributes on these tags are dropped without warning.
   This is also why you cannot take the number of attempts for a :class:`py_trees.decorators.Retry` from a port value via XML ---
   the parser only wires ports (and forwards constructor kwargs; see :ref:`ports-xml-attributes-label` above) on resolved classes that derive from
   :class:`~py_trees.ports.PortsMixin` (whether resolved via the ``"auto"`` registry or an explicit ``node_registry`` dict; see :ref:`ports-xml-class-resolution-label` above).
   Built-in decorators aren't recognised as XML tags at all.

**3. The pattern: port-aware wrapper classes.**

   To make an existing upstream behaviour, decorator, or composite port-aware, wrap it in a small adapter class that combines :class:`~py_trees.ports.PortsMixin` with the upstream class and reads its runtime parameters from input ports.

   The recommended naming is to keep the short upstream class name (``Retry``, ``Repeat``, ``Parallel``, …) and place the port-aware version under a ``ports`` submodule that mirrors the upstream layout (e.g. ``py_trees.ports.decorators.Retry`` alongside the upstream ``py_trees.decorators.Retry``).
   The import path carries the "ported" information, so the class name stays short and matches its upstream counterpart.

   Example: a port-aware :class:`~py_trees.decorators.Retry` that takes its ``num_failures`` from an input port --- intended to live in ``py_trees.ports.decorators`` when contributed upstream, or in your own project's ``ports`` submodule:

   .. code-block:: python

      # py_trees/ports/decorators.py  (or yourproject/ports/decorators.py)
      import py_trees
      from py_trees.ports import PortInformation, PortsMixin

      class Retry(PortsMixin, py_trees.decorators.Retry):
          """Retry that reads its failure budget from an input port."""

          INPUT_PORTS = {"num_failures": PortInformation(data_type=int, required=True)}
          OUTPUT_PORTS = {}

          def __init__(
              self,
              name: str,
              child: py_trees.behaviour.Behaviour,
              **kwargs,
          ):
              # Start with a safe default; it is overwritten on every initialise().
              super().__init__(
                  name=name, child=child, num_failures=1, **kwargs
              )

          def initialise(self) -> None:
              # Read the port value at tick boundary and apply it before the
              # upstream Retry logic runs.
              self.num_failures = self.get_input("num_failures")
              super().initialise()

   Users import it as ``from py_trees.ports.decorators import Retry`` (or the equivalent path in their own project) --- the import path disambiguates it from the upstream ``py_trees.decorators.Retry``, so the class name stays clean.

   The XML then can accept the input via (remapped) ports::

      <Retry name="retry" num_failures="{retry_budget}">
          <SomeBehaviour name="worker" />
      </Retry>

.. note:: Contributing port-enabled extensions upstream is encouraged!

   If you build a generally useful port-aware wrapper --- for example, a port-enabled :class:`~py_trees.decorators.Retry`, :class:`~py_trees.decorators.Repeat`, :class:`~py_trees.composites.Parallel`, or :class:`~py_trees.timers.Timer` --- please consider contributing it back to py_trees under the matching ``py_trees.ports.*`` subpackage (``py_trees.ports.decorators``, ``py_trees.ports.composites``, ``py_trees.ports.timers``, and so on, mirroring the upstream module layout).
   A shared library of canonical port-aware adapters saves every user from re-implementing the same wrappers.
   Open a PR against the `py_trees devel branch <https://github.com/splintered-reality/py_trees>`_ and we will happily review it.

.. _ports-demos-section-label:

Demos
-----

Example programs live in ``py_trees.demos.ports``.
Each demo module has a top-level docstring describing the scenario it demonstrates.

Module reference
----------------

.. automodule:: py_trees.ports
   :members:
   :show-inheritance:
   :synopsis: typed input/output ports

.. automodule:: py_trees.parsers.behaviour_tree_xml
   :members:
   :show-inheritance:
   :synopsis: experimental XML parser for ports

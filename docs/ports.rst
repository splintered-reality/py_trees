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
  A node's ``input_ports()`` and ``output_ports()`` declarations *are* its data-flow API.
  A reader can see at a glance what a node consumes and produces without reading through its :meth:`~py_trees.behaviour.Behaviour.update` method.

* **Structured, early error detection.**
  Port values are type-checked at runtime (``TypeError`` on mismatched writes/reads), required inputs without data raise :class:`~py_trees.ports.NoDataAvailable` instead of returning ``None`` silently, and misconfiguration (a remap for a port that isn't declared, or a port type that can't accept the wiring) surfaces at setup time rather than deep in a tick.

* **XML authoring.**
  Once a library of port-enabled nodes exists, trees become declarative data.
  Non-programmers can read and edit tree structure (and the data wiring) without touching Python.
  See the :ref:`XML parser section <py-trees-demo-ports-xml-tree-program>` below.

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
       @classmethod
       def input_ports(cls):
           return {
               "a": PortInformation(type=float, required=True),
               "b": PortInformation(type=float, required=True),
           }

       @classmethod
       def output_ports(cls):
           return {
               "product": PortInformation(type=float, required=True),
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

Experimental XML parser
-----------------------

The module :mod:`py_trees.parsers.behaviour_tree_xml` ships an (experimental) parser for the `BehaviorTree.CPP <https://www.behaviortree.dev/docs/learn-the-basics/main_concepts>`_ XML format.
It builds a py_trees tree from an XML file and auto-generates the port remappings for every node.

.. code-block:: python

   from py_trees.parsers.behaviour_tree_xml import parse_behaviour_tree_xml

   root = parse_behaviour_tree_xml(
       "my_tree.xml",
       init_lookup={"MyNode": MyNode, "OtherNode": OtherNode, ...},
   )

See the :ref:`demos <ports-demos-section-label>` below for working examples.

.. _ports-xml-attributes-label:

XML attributes: ports *and* constructor arguments
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Attributes on a node's XML tag serve **two** distinct purposes:

1. Attribute names that match a declared port (``input_ports()`` or ``output_ports()``) are treated as **port remappings**.
   Values may be ``{curly_key}`` references (wired to the remapping table) or literal constants (type-converted according to the port's declared type).

2. Attribute names that do **not** match any declared port are treated as **constructor keyword arguments** and forwarded to the class constructor.
   Values are type-converted based on the constructor's type annotations (strings to ``int`` / ``float`` / ``bool`` / enum / etc.); un-annotated parameters receive the raw string.
   ``{curly_key}`` references are *not* allowed for constructor kwargs --- they raise a ``ValueError`` at parse time, because constructor kwargs can't be re-wired at runtime the way ports can.

Example::

   class Greeting(BehaviourWithPorts):

       @classmethod
       def input_ports(cls):
           return {"name_key": PortInformation(type=str, required=True)}

       @classmethod
       def output_ports(cls):
           return {"greeting": PortInformation(type=str, required=True)}

       def __init__(self, name: str, prefix: str = "Hello", **kwargs):
           super().__init__(name=name, **kwargs)
           self.prefix = prefix        # not a port; populated from XML attribute

       def update(self):
           self._set_output("greeting", f"{self.prefix}, {self.get_input('name_key')}!")
           return py_trees.common.Status.SUCCESS

.. code-block:: xml

   <Greeting name="hello_node" name_key="{target}" prefix="Howdy"/>
   <!--       ^^^ behaviour name   ^^^ port remap    ^^^ ctor kwarg  -->

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
   This is also why you cannot take the number of attempts for a :class:`py_trees.decorators.Retry` from a port value via XML --- the parser only wires ports (and forwards constructor kwargs; see :ref:`ports-xml-attributes-label` above) on classes registered in ``init_lookup`` that derive from :class:`~py_trees.ports.PortsMixin`.
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

          @classmethod
          def input_ports(cls):
              return {"num_failures": PortInformation(type=int, required=True)}

          @classmethod
          def output_ports(cls):
              return {}

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

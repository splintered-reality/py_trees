.. _ports-section-label:

Ports (Experimental)
====================

.. warning::

   The ports module (:mod:`py_trees.ports`) and the XML parser
   (:mod:`py_trees.parsers.behaviour_tree_xml`) are **experimental**.
   Their API may change between releases.

Overview
--------

**Ports** add a structured way of wiring up data exchange between nodes.
Each node defines the data it reads and writes as **input** and **output**
ports. The ports are wired to blackboard keys (the *remapping*), and port
values are type-checked at runtime. This allows for a defined and constrained
way of data exchange between nodes which is less error-prone and easier to debug
than simply writing data on the blackboard and letting nodes read from and
write to that entry directly.

The primary API is :class:`py_trees.ports.PortsMixin`.  Concrete nodes
typically inherit from the convenience base
:class:`py_trees.ports.BehaviourWithPorts`, which combines the mixin with
:class:`py_trees.behaviour.Behaviour`:

.. code-block:: python

   import py_trees
   from py_trees.ports import BehaviourWithPorts

   class Multiply(BehaviourWithPorts):
       @classmethod
       def input_ports(cls):
           return {
               "a": (float, True),   # (type, required)
               "b": (float, True),
           }

       @classmethod
       def output_ports(cls):
           return {
               "product": (float, True),
           }

       def update(self):
           self._set_output("product", self.get_input("a") * self.get_input("b"))
           return py_trees.common.Status.SUCCESS

Wiring (remapping) and type checking
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Ports become usable after :meth:`~py_trees.ports.PortsMixin.setup_ports`
has been called with the *port remappings* (the "wiring").  Remappings
map each port to an absolute or relative blackboard key.  When a port is
*not* remapped, an automatic UUID-scoped storage key is generated so
sibling nodes never collide by accident.
However, the main purpose of remapping is to "wire" one node's output ports
to another node's input ports so they can exchange data, which means you'll
generally want to remap at least the ports that carry data the nodes need to
exchange.

.. code-block:: python

   node = Multiply(name="mul")
   node.setup_ports(
       port_remappings={
           "a":       "/numbers/a",
           "b":       "/numbers/b",
           "product": "/numbers/product",
       }
   )

In the example above, another node's output ports would then be remapped to
``/numbers/a`` and ``/numbers/b`` and thereby provide the input for the ``Multiply`` node.

Why is ``setup_ports()`` a separate call?  Because the remapping table
usually cannot be computed until the entire tree topology is known —
either the user assembles it by hand or a parser generates it from
e.g. XML (more on that next).  See :class:`py_trees.ports.PortsMixin` for the full contract
and semantics.

Experimental XML parser
-----------------------

The module :mod:`py_trees.parsers.behaviour_tree_xml` ships an
(experimental) parser for the `BehaviorTree.CPP
<https://www.behaviortree.dev/docs/learn-the-basics/main_concepts>`_
XML format.  It builds a py_trees tree from an XML file and
auto-generates the port remappings for every node.

.. code-block:: python

   from py_trees.parsers.behaviour_tree_xml import parse_behaviour_tree_xml

   root = parse_behaviour_tree_xml(
       "my_tree.xml",
       init_lookup={"MyNode": MyNode, "OtherNode": OtherNode, ...},
   )

See the :ref:`demos <ports-demos-section-label>` below for working
examples.

.. _ports-demos-section-label:

Demos
-----

Four demos are shipped with the library.  Each demo has a CLI entry
point installed with the package.

.. list-table::
   :header-rows: 1
   :widths: 30 70

   * - Entry point
     - Description
   * - ``py-trees-demo-ports-basic``
     - Single behaviour with typed ports and explicit remapping.
   * - ``py-trees-demo-ports-remapping``
     - Two isolated 3-step pipelines reusing the same port names,
       separated by subtree namespaces.
   * - ``py-trees-demo-ports-xml-tree``
     - Parse a behaviour tree from XML with a subtree and a remapped
       final value.
   * - ``py-trees-demo-ports-nested-subtrees``
     - A nested-subtree robot pickup mission, showing how subtree
       port remappings propagate through multiple layers.

Run any of them from the shell after ``poetry install``:

.. code-block:: bash

   py-trees-demo-ports-basic
   py-trees-demo-ports-remapping
   py-trees-demo-ports-xml-tree
   py-trees-demo-ports-nested-subtrees

Alternatively, without the entry point:

.. code-block:: bash

   python -m py_trees.demos.ports.basic
   python -m py_trees.demos.ports.remapping
   python -m py_trees.demos.ports.xml_tree
   python -m py_trees.demos.ports.nested_subtrees

.. _py-trees-demo-ports-basic-program:

py-trees-demo-ports-basic
~~~~~~~~~~~~~~~~~~~~~~~~~

A single :class:`~py_trees.ports.BehaviourWithPorts` node that reads two
floats, multiplies them, and writes the product to an explicitly
remapped blackboard key.  The demo seeds the inputs on the blackboard,
ticks the node once, and prints the three values.

.. literalinclude:: ../py_trees/demos/ports/basic.py
   :language: python
   :linenos:
   :caption: py_trees/demos/ports/basic.py

.. _py-trees-demo-ports-remapping-program:

py-trees-demo-ports-remapping
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Two independent 3-step pipelines (``GenerateValue`` → ``AppendSuffix``
→ ``ReadResult``) are run in separate subtree namespaces
(``/pipeline_a`` and ``/pipeline_b``).  Both pipelines use the **same**
port names internally; the subtree namespace keeps them fully
isolated.  The demo also shows that an unremapped output (the
``value`` status field on ``AppendSuffix``) gets its own
UUID-scoped default key so it does not clash with the pipeline wiring.

.. literalinclude:: ../py_trees/demos/ports/remapping.py
   :language: python
   :linenos:
   :caption: py_trees/demos/ports/remapping.py

.. _py-trees-demo-ports-xml-tree-program:

py-trees-demo-ports-xml-tree
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

A small behaviour tree is loaded from XML.  A subtree
``ComposeGreeting`` produces a greeting and appends a suffix; the
main tree wires the subtree's result to a consumer that prints it.
This demo exercises the parser's support for ``<SubTree>`` remapping,
port resolution, and constant (non-``{key}``) attribute values.

.. literalinclude:: ../py_trees/demos/ports/xml_tree.py
   :language: python
   :linenos:
   :caption: py_trees/demos/ports/xml_tree.py

.. literalinclude:: ../py_trees/demos/ports/xml_tree.xml
   :language: xml
   :linenos:
   :caption: py_trees/demos/ports/xml_tree.xml

.. _py-trees-demo-ports-nested-subtrees-program:

py-trees-demo-ports-nested-subtrees
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

A robotics-style pickup mission built from nested subtrees
(``PickupFromTableRoutine`` which in turn uses ``ArmPickupRoutine``).
Each subtree has an explicit port contract documented in the XML.
The demo shows how port remappings propagate through multiple
layers of subtrees while internal keys remain local to their subtree.

.. literalinclude:: ../py_trees/demos/ports/nested_subtrees.py
   :language: python
   :linenos:
   :caption: py_trees/demos/ports/nested_subtrees.py

.. literalinclude:: ../py_trees/demos/ports/nested_subtrees.xml
   :language: xml
   :linenos:
   :caption: py_trees/demos/ports/nested_subtrees.xml

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

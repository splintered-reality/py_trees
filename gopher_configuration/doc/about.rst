About
=====

Motivation
----------

We typically configure every node privately either via param tags, yaml files or remaps. This creates
a few headaches that this package attempts to try and solve by creating a core set of configurations
for a gopher. In order of importance, the problems include:

* Processes with many independent modules (e.g. behaviour trees) standardising parameters for patterns, sounds, topics, frames.
* Processes with many independent modules (e.g. behaviour trees) working out what parameters it should set in launch files.
* Standardising particular gopher topics/frames/interactions - e.g. what topic should represent honk/go button? what frame is the global one?
* Excessive and redundant parameterisations/remaps for gopher specific software in launch files.

Concepts
--------

This package defines core configurations for

* ros communications: topics, services, namespaces, frames. It also
* namespaces : root locations for a large set of communications or the parameter server
* interactions : sounds, buttons, led patterns

It also lets you define any other configurations you wish for the purposes of experimentation or
customisation of a particular gopher.

Demo
----

Simply run the ``gopher_configuration`` script with no ros running. This will load the defaults into an instance
of the :class:`.Configuration` class and display them when it fails to find a rosparam server.

.. code-block:: bash
   
   $ gopher_configuration

Create a Configuration
----------------------

The defaults for a gopher are a good starting point.

.. literalinclude:: ../param/defaults.yaml
   :language: yaml
   :linenos:

You can then create your own customisation of this yaml in a separate file. Just extract the elements you wish to override and
add any extra elements you'd care to define.

**Load configuration onto the parameter server**

Include your extensions/overrides of the defaults in a custom yaml configuration loaded on top of the defaults.

.. code-block:: xml
   :linenos:

   <launch>
       <rosparam ns="/gopher/configuration" command="load" file="$(find gopher_configuration)/param/defaults.yaml"/>
       <rosparam ns="/gopher/configuration" command="load" file="$(find foo_configuration)/param/customisation.yaml"/>
   <launch>

View a Configuration
--------------------

To view the current runtime configuration simply run the command line program:

.. code-block:: bash
   
   $ gopher_configuration

Use a Configuration - Python
----------------------------

Python access to this configuration is provided via the :class:`.Configuration` class. See that class itself
for more information and usage instructions.

Use a Configuration - C++
-------------------------

TODO


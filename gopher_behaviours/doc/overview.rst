Overview
========

A brief overview of the folders & files in this package.


Scripts
-------

This folder contains a variety of scripts to make the robot do stuff like deliveries, run demos or help with getting
through doors.

The important ones:

* ``gopher_deliveries``
   This is a ROS node loading up the behaviour tree required for Gophers to be able to do deliveries.

* ``gopher_delivery_run``
   A command-line tool to trigger a delivery run across specified locations. Use the ``--help`` option
   to find out how to use it.


Launch
------

Here you find launchers for the scripts above, most importantly ``gopher_deliveries.launch``, which starts the
``gopher_delivery`` script.
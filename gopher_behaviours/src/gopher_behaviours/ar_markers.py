#!/usr/bin/env python
#
# License: Yujin
#
##############################################################################
# Description
##############################################################################

"""
.. module:: ar_markers
   :platform: Unix
   :synopsis: Working with the ar marker node

Oh my spaghettified magnificence,
Bless my noggin with a tickle from your noodly appendages!

----

"""

##############################################################################
# Imports
##############################################################################

import dynamic_reconfigure.client
import py_trees
import rospy

##############################################################################
# Behaviours
##############################################################################


class ControlARMarkerTracker(py_trees.Behaviour):
    """
    Behaviour for enabling/disabling an AR marker tracker
    """
    def __init__(self, name, topic, enable=True):
        """
        Put together the behaviour
        """
        super(ControlARMarkerTracker, self).__init__(name)
        self._topic = topic
        self._enable = enable
        self._dyn_reconf_client_ar_tracker = None

    def _ar_tracker_control(self, enable):
        if enable:
            params = {'enabled': 'true'}
        else:
            params = {'enabled': 'false'}
        if self._dyn_reconf_client_ar_tracker is not None:
            self._dyn_reconf_client_ar_tracker.update_configuration(params)

    def setup(self, timeout):
        """
        Try and connect to the dynamic reconfigure server.
        """
        if not self._dyn_reconf_client_ar_tracker:
            try:
                self._dyn_reconf_client_ar_tracker = dynamic_reconfigure.client.Client(
                    self._topic,
                    timeout=timeout
                )
            except rospy.ROSException:
                rospy.logwarn("Behaviours [%s" % self.name + "]: could not connect to dynamic reconfigure server [%s][%s secs]" % (self._topic, timeout))
                return False
        return True

    def update(self):
        """
        Called by the behaviour :py:meth:`tick() <py_trees.behaviours.Behaviour.tick>` function.
        Does the real work...
        """
        if self._dyn_reconf_client_ar_tracker:
            if self.get_status() != py_trees.Status.RUNNING:
                self._ar_tracker_control(self._enable)
                return py_trees.Status.RUNNING
            else:
                return py_trees.Status.SUCCESS
        else:
            rospy.logwarn("Behaviours [%s" % self.name + "]: failed to connect to dynamic reconfigure server")
            return py_trees.Status.FAILURE

    def stop(self, new_status=py_trees.Status.INVALID):
        """
        Make sure to disable it if we are responsible for starting it.
        """
        super(ControlARMarkerTracker, self).stop(new_status)
        if new_status == py_trees.Status.INVALID:
            if self._enable:
                self._ar_tracker_control(False)

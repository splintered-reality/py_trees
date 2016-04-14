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
import gopher_configuration
import py_trees
import rospy

##############################################################################
# Conveniences
##############################################################################


def create_ar_tracker_pair_blackboxes():
    """
    Sets up blackboxes for enabling long/short range ar trackers on gophers.
    :returns: 2-tuple for on/off blackboxes.
    """
    gopher = gopher_configuration.Configuration(fallback_to_defaults=True)

    ar_tracker_on_long_range = ControlARMarkerTracker("Long-Range AR Tracker On",
                                                      gopher.topics.ar_tracker_long_range,
                                                      True)
    ar_tracker_on_short_range = ControlARMarkerTracker("Short-Range AR Tracker On",
                                                       gopher.topics.ar_tracker_short_range,
                                                       True)
    ar_tracker_off_long_range = ControlARMarkerTracker("Long-Range AR Tracker Off",
                                                       gopher.topics.ar_tracker_long_range,
                                                       False)
    ar_tracker_off_short_range = ControlARMarkerTracker("Short-Range AR Tracker Off",
                                                        gopher.topics.ar_tracker_short_range,
                                                        False)

    ar_tracker_on = py_trees.composites.Sequence(name="Ar Markers On")
    ar_tracker_on.blackbox_level = py_trees.common.BlackBoxLevel.DETAIL
    ar_tracker_off = py_trees.composites.Sequence(name="Ar Markers Off")
    ar_tracker_off.blackbox_level = py_trees.common.BlackBoxLevel.DETAIL

    ar_tracker_on.add_children([ar_tracker_on_long_range, ar_tracker_on_short_range])
    ar_tracker_off.add_children([ar_tracker_off_long_range, ar_tracker_off_short_range])
    return (ar_tracker_on, ar_tracker_off)

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
            try:
                self._dyn_reconf_client_ar_tracker.update_configuration(params)
            except rospy.service.ServiceException as e:
                rospy.logwarn("Behaviours [%s" % self.name + "]: caught exception from the ar marker service, probably just shutdown issues [%s]" % str(e))

    def setup(self, timeout):
        """
        Try and connect to the dynamic reconfigure server.
        """
        self.logger.debug("  %s [ARMarkers::setup()]" % self.name)
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
            self._ar_tracker_control(self._enable)
            return py_trees.Status.SUCCESS
        else:
            rospy.logerr("Behaviours [%s" % self.name + "]: failed to connect to dynamic reconfigure server")
            return py_trees.Status.FAILURE

    def terminate(self, new_status):
        """
        Terminating - even if you have an 'off' ar markers instance elsewhere, it is
        important to disable them here since your tree may go down before you get to the 'off'.
        The alternative requires you to put in complicated wiring - easier just to multiply disable
        to be sure.
        """
        if new_status == py_trees.Status.INVALID:
            if self._enable:
                self._ar_tracker_control(False)

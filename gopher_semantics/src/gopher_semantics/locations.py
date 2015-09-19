#!/usr/bin/env python
#
# License: Unspecified
#
##############################################################################
# Imports
##############################################################################

import geometry_msgs.msg as geometry_msgs
import gopher_semantic_msgs.msg as gopher_semantic_msgs
import rocon_console.console as console
import rospy

##############################################################################
# Classes
##############################################################################


class Locations(dict):
    '''
    Repository of semantic map locations. Each map location should be provided in yaml
    in the following form:

    .. code-block:: yaml

       semantic_locations:
         beer_fridge:
           name: Beer Fridge
           description: No beer? Please rectify immediately.
           pose:
             x: 1.500000
             y: 0.000000
             theta: 0.000000
           keyframe_id: 2

    Parameters:

    - ~semantics_parameter_namespace : where it can find semantics information on the ros parameter server.
    '''
    def __init__(self, semantics_parameter_namespace=None):
        super(Locations, self).__init__()
        if semantics_parameter_namespace is None:
            semantics_parameter_namespace = rospy.get_param('~semantics_parameter_namespace', rospy.resolve_name('~'))
        parameters = rospy.get_param(semantics_parameter_namespace + "/locations", {})
        # legacy support
        if not parameters:
            parameters = rospy.get_param(semantics_parameter_namespace + "/semantic_locations", {})
        for unique_name, details in parameters.iteritems():
            location = gopher_semantic_msgs.Location()
            location.unique_name = unique_name
            try:
                location.name = details['name']
                location.description = details['description']
                location.pose = geometry_msgs.Pose2D()
                location.pose.x = details['pose']['x']
                location.pose.y = details['pose']['y']
                location.pose.theta = details['pose']['theta']
                location.keyframe_id = details['keyframe_id']
                location.world = details['world'] if 'world' in details else 'default'
                self.__setitem__(unique_name, location)
            except KeyError as e:
                rospy.logwarn("Map Locations : one of the expected fields for location '%s' was missing! [%s]" % (unique_name, e))

    def __str__(self):
        s = console.bold + "\nLocations:\n" + console.reset
        for name in sorted(self):
            s += console.green + "  %s\n" % name
            msg = dict.__getitem__(self, name)
            for key in msg.__slots__:
                value = getattr(msg, key)
                if isinstance(value, geometry_msgs.Pose2D):
                    s += console.cyan + "    %s: " % key + console.yellow + "{ x: %s y: %s theta: %s }\n" % (value.x, value.y, value.theta)
                else:
                    s += console.cyan + "    %s: " % key + console.yellow + "%s\n" % (value if value is not None else '-')
        s += console.reset
        return s

    def to_msg(self):
        msg = gopher_semantic_msgs.Locations()
        msg.locations = []
        for location_msg in self.values():
            msg.locations.append(location_msg)
        return msg

    def spin(self):
        rospy.spin()

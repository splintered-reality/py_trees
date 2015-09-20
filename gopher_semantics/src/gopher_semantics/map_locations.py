#!/usr/bin/env python
#
# License: Unspecified
#
##############################################################################
# Imports
##############################################################################

import rospy
import sys

import geometry_msgs.msg as geometry_msgs
from gopher_std_msgs.msg import Location
from gopher_std_msgs.msg import Locations

##############################################################################
# Classes
##############################################################################


class SemanticLocations():
    """
    This is a borg like entity currently used by the gopher behaviours
    (work in progress).
    """
    __shared_state = {}

    def loc_cb(self, msg):
        for location in msg.locations:
            self.__shared_state[location.unique_name] = location

        self.__dict__ = self.__shared_state

    def __init__(self):
        # subscriber for each instance? should really be a shared subscriber,
        # but can't initialise a subscriber without doing node init, so might be
        # impossible.
        self.semantic_locs = rospy.Subscriber('~semantic_locations', Locations, self.loc_cb)

    def __repr__(self):
        return "<SemanticLocations __shared_state:%s>" % self.__shared_state

    # Allow the use of this object like a dict. We can access the elements of
    # the shared dict using the [] operator on the object.
    def __getitem__(self, item):
        return self.__shared_state[item]


class MapLocations(object):
    '''
    .. deprecated::
       Shifting to MapLocations instead.
    '''
    def __init__(self):
        self.loc_publisher = rospy.Publisher('semantic_locations', Locations, latch=True, queue_size=1)

        # get the keys which refer to each location so that we don't have to
        # retrieve parameters multiple times to check things
        try:
            location_keys = rospy.get_param('~semantic_locations')
        except KeyError:
            rospy.logerr("Map Locations : could not find semantic location keys!")
            sys.exit("could not find semantic location keys")

        msg = Locations()
        locs = []
        for key in sorted(location_keys, key=location_keys.get):
            new_location = Location()
            new_location.unique_name = key
            try:
                new_location.name = location_keys[key]['name']
                new_location.description = location_keys[key]['description']
                new_location.pose = geometry_msgs.Pose2D()
                new_location.pose.x = location_keys[key]['pose']['x']
                new_location.pose.y = location_keys[key]['pose']['y']
                new_location.pose.theta = location_keys[key]['pose']['theta']
                new_location.keyframe_id = location_keys[key]['keyframe_id']
            except KeyError:
                rospy.logwarn("Map Locations : one of the expected fields for key %s was missing!" % key)

            locs.append(new_location)

        msg.locations = locs

        self.loc_publisher.publish(msg)

    def spin(self):
        rospy.spin()

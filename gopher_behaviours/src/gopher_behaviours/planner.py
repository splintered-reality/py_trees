#!/usr/bin/env python

##############################################################################
# Imports
##############################################################################

import rospy
import rocon_python_comms
import delivery
import moveit
import gopher_std_msgs.msg as gopher_std_msgs

##############################################################################
# Implementation
##############################################################################

class Planner():

    def __init__(self, auto_go):
        self.auto_go = auto_go
        self.semantic_locations = {}

        semantic_locations = []  # fill this up from the map locations server
        try:
            response = rocon_python_comms.SubscriberProxy('/navi/semantic_locations', gopher_std_msgs.Locations)(rospy.Duration(5))  # TODO validate this is long enough for our purposes
            if response is not None:
                rospy.loginfo("Gopher Deliveries Planner : served semantic locations from the map locations server [%s]" % [location.unique_name for location in response.locations])
                semantic_locations = response.locations
        except rospy.exceptions.ROSInterruptException:  # make sure to handle a Ros shutdown
            rospy.logwarn("Gopher Deliveries Planner : ros shutdown(?) while attempting to connect to the map locations server")
            return
        for semantic_location in semantic_locations:
            self.semantic_locations[semantic_location.unique_name] = semantic_location
        rospy.logdebug("Gopher Deliveries Planner : semantic locations served: \n%s" % self.semantic_locations)
        
    def create_tree(self, locations, undock=True):
        """
        Find the semantic locations corresponding to the incoming string location identifier and
        create the appropriate behaviours.

        :param: string list of location unique names given to us by the delivery goal.

        .. todo::

           Clean up the key error handling
        """
        # if we are constructing a "complete" 
        children = [moveit.UnDock("UnDock")] if undock else []
        for ind, location in enumerate(locations):
            try:
                semantic_location = self.semantic_locations[location]  # this is the full gopher_std_msgs.Location structure
                children.append(moveit.MoveToGoal(name=semantic_location.name, pose=semantic_location.pose))
                # don't append a waiting action for the last location
                if ind < len(locations) - 1:
                    children.append(
                        delivery.Waiting(name="Waiting at " + semantic_location.name,
                                         location=semantic_location.unique_name,
                                         dont_wait_for_hoomans_flag=self.auto_go)
                    )
            except KeyError, ke:  # a location passed was unknown : ignore it
                rospy.logwarn("{0} is not in semantic_locations".format(location))
                continue

        return children


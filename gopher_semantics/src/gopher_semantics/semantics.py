#
# License: Yujin
#
##############################################################################
# Description
##############################################################################

"""
.. module:: semantics
   :platform: Unix
   :synopsis: Frontend to the semantic locations database.

A general semantics yaml configuration can be parsed and loaded (loads
individual modules as needed depending on their presence in the yaml).
----

"""

##############################################################################
# Imports
##############################################################################

import gopher_semantic_msgs.msg as gopher_semantic_msgs
import rocon_python_comms.utils
import rospy
import std_msgs.msg as std_msgs

from .docking_stations import DockingStations
from .elevators import Elevators
from .locations import Locations
from .parameters import Parameters
from .worlds import Worlds

##############################################################################
# Semantics
##############################################################################


class Semantics(object):
    '''
    Frontend to the gopher semantics, also publishes every item and
    exposes an api to the underlying data.

    Parameters:

    - ~semantics_parameter_namespace : where it can find semantics information on the ros parameter server.

    Publishers:

    - ~docking_stations [gopher_semantic_msgs.DockingStations]
    - ~elevators [gopher_semantic_msgs.Elevators]
    - ~locations [gopher_semantic_msgs.Locations]
    - ~worlds [gopher_semantic_msgs.World]
    '''
    def __init__(self, semantics_parameter_namespace=None, create_publishers=False):
        """
        :param str semantics_parameter_naemspace: this is where you can find the parameters on the rosparam serer
        :param bool create_publishers: only the node is interested in doing this.
        """
        # right now, overkill for one bloody variable...
        self.parameters = Parameters(semantics_parameter_namespace)
        semantics = rospy.get_param(self.parameters.semantics_parameter_namespace, {})

        # assemble variables
        publisher_details = []
        latched = True
        queue_size_five = 5
        self.semantic_modules = {}
        # special case - supporting legacy
        if 'semantic_locations' in semantics or 'locations' in semantics:
            self.semantic_modules['locations'] = Locations(self.parameters.semantics_parameter_namespace)
        if 'worlds' in semantics:
            self.semantic_modules['worlds'] = Worlds(self.parameters.semantics_parameter_namespace)
        if 'docking_stations' in semantics:
            self.semantic_modules['docking_stations'] = DockingStations(self.parameters.semantics_parameter_namespace)
        if 'elevators' in semantics:
            self.semantic_modules['elevators'] = Elevators(self.parameters.semantics_parameter_namespace)
        for module_name, module in self.semantic_modules.iteritems():
            setattr(self, module_name, module)

        if create_publishers:
            # special case - supporting legacy
            if 'semantic_locations' in semantics or 'locations' in semantics:
                publisher_details.append(('~locations', gopher_semantic_msgs.Locations, latched, queue_size_five))
            if 'worlds' in semantics:
                publisher_details.append(('~worlds', gopher_semantic_msgs.Worlds, latched, queue_size_five))
            if 'docking_stations' in semantics:
                publisher_details.append(('~docking_stations', gopher_semantic_msgs.DockingStations, latched, queue_size_five))
            if 'elevators' in semantics:
                publisher_details.append(('~elevators', gopher_semantic_msgs.Elevators, latched, queue_size_five))

            publisher_details.append(('~introspection/parameters', std_msgs.String, latched, queue_size_five)),
            publisher_details.append(('~introspection/semantics', std_msgs.String, latched, queue_size_five)),
            self.publishers = rocon_python_comms.utils.Publishers(publisher_details)

            # validate
            # TODO check if there are any locations assigned to worlds, not in worlds

            # publish
            semantics_string = std_msgs.String()
            for name, module in sorted(self.semantic_modules.iteritems()):
                self.publishers.__dict__[name].publish(module.to_msg())
                semantics_string.data += "%s" % module
            self.publishers.parameters.publish(std_msgs.String("%s" % self.parameters))
            self.publishers.semantics.publish(semantics_string)

    def spin(self):
        rospy.spin()

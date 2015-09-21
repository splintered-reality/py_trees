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
import os
import rocon_python_comms.utils
import rospkg
import rospy
import std_msgs.msg as std_msgs
import yaml

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
    def __init__(self, semantics_parameter_namespace=None, create_publishers=False, from_yaml=None):
        """
        :param str semantics_parameter_naemspace: this is where you can find the parameters on the rosparam serer
        :param bool create_publishers: only the node is interested in doing this.
        :param str from_yaml: load from yaml instead of the rosparam server.

        .. todo::

           Might be worth splitting this into a yaml parent vs ros child class with all the extra bells & whistles
        """
        if from_yaml is not None:
            self.parameters = None
            semantics = yaml.load(open(from_yaml))
        else:
            self.parameters = Parameters(semantics_parameter_namespace)
            semantics = rospy.get_param(self.parameters.semantics_parameter_namespace, {})

        #######################################
        # Modules
        #######################################
        self.semantic_modules = {}
        if 'locations' in semantics:
            self.semantic_modules['locations'] = Locations(from_yaml_object=semantics["locations"]) if from_yaml else Locations(self.parameters.semantics_parameter_namespace)
        # special case - supporting legacy
        if 'semantic_locations' in semantics:
            self.semantic_modules['locations'] = Locations(from_yaml_object=semantics["semantic_locations"]) if from_yaml else Locations(self.parameters.semantics_parameter_namespace)
        if 'worlds' in semantics:
            self.semantic_modules['worlds'] = Worlds(from_yaml_object=semantics["worlds"]) if from_yaml else Worlds(self.parameters.semantics_parameter_namespace)
        if 'docking_stations' in semantics:
            self.semantic_modules['docking_stations'] = DockingStations(from_yaml_object=semantics["docking_stations"]) if from_yaml else DockingStations(self.parameters.semantics_parameter_namespace)
        if 'elevators' in semantics:
            self.semantic_modules['elevators'] = Elevators(from_yaml_object=semantics["elevators"]) if from_yaml else Elevators(self.parameters.semantics_parameter_namespace)
        for module_name, module in self.semantic_modules.iteritems():
            setattr(self, module_name, module)

        #######################################
        # Publishers
        #######################################
        if create_publishers:
            publisher_details = []
            latched = True
            queue_size_five = 5
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
            for name, module in sorted(self.semantic_modules.iteritems()):
                self.publishers.__dict__[name].publish(module.to_msg())
            if self.parameters is not None:
                self.publishers.parameters.publish(std_msgs.String("%s" % self.parameters))
            self.publishers.semantics.publish("%s" % self)

    def __str__(self):
        s = ""
        for unused_name, module in sorted(self.semantic_modules.iteritems()):
            s += "%s" % module
        return s

    def spin(self):
        rospy.spin()


##############################################################################
# Desirables
##############################################################################

def load_desirable_destinations():
    """
    Convenience function for loading our desirables from yaml directly. This
    is useful for alot of testing.
    """
    rospack = rospkg.RosPack()
    gopher_semantics_path = rospack.get_path('gopher_semantics')
    filename = os.path.join(gopher_semantics_path, 'param', 'desirable_destinations.yaml')
    return Semantics(from_yaml=filename)

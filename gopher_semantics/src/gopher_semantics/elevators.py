#
# License: Yujin
#
##############################################################################
# Description
##############################################################################

"""
.. module:: elevators
   :platform: Unix
   :synopsis: Semantics for elevators.

Oh my spaghettified magnificence,
Bless my noggin with a tickle from your noodly appendages!

----

"""

##############################################################################
# Imports
##############################################################################

import geometry_msgs.msg as geometry_msgs
import gopher_semantic_msgs.msg as gopher_semantic_msgs
import rocon_console.console as console
import rospy

##############################################################################
# Elevators
##############################################################################


class Elevators(dict):
    '''
    Repository of docking station knowledge. Each docking station should be provided in yaml
    in the following form:

    .. code-block:: yaml

       elevators:
         highway_to_hell:
           name: 'Highway to Hell'
           levels:
             - world: heaven
               floor: 999
               entry: { x: 0.00, y: 0.00, theta: 0.00 }
               exit:  { x: 0.00, y: 0.00, theta: 0.00 }
             - world: hell
               floor: 666
                entry: { x: 0.00, y: 0.00, theta: 0.00 }
               exit:  { x: 0.00, y: 0.00, theta: 0.00 }
             - world: earth
               floor: 1
               entry: { x: 0.00, y: 0.00, theta: 0.00 }
               exit:  { x: 0.00, y: 0.00, theta: 0.00 }

    Parameters:

    - ~semantics_parameter_namespace : where it can find semantics information on the ros parameter server.
    '''
    def __init__(self, semantics_parameter_namespace=None, from_yaml_object=None):
        """
        Load from the rosparam server or from yaml object.
        """
        super(Elevators, self).__init__()
        data = from_yaml_object
        if data is None:
            # look to the rosparam server
            if semantics_parameter_namespace is None:
                semantics_parameter_namespace = rospy.get_param('~semantics_parameter_namespace', rospy.resolve_name('~'))
            data = rospy.get_param(semantics_parameter_namespace + "/elevators", {})
        for unique_name, fields in data.iteritems():
            try:
                elevator = gopher_semantic_msgs.Elevator()
                elevator.unique_name = unique_name
                elevator.name = fields['name']
                for level in fields['levels']:
                    elevator_level = gopher_semantic_msgs.ElevatorLevel()
                    elevator_level.world = level['world']
                    elevator_level.floor = level['floor']
                    elevator_level.entry = geometry_msgs.Pose2D()
                    elevator_level.entry.x = level['entry']['x']
                    elevator_level.entry.y = level['entry']['y']
                    elevator_level.entry.theta = level['entry']['theta']
                    elevator_level.exit = geometry_msgs.Pose2D()
                    elevator_level.exit.x = level['exit']['x']
                    elevator_level.exit.y = level['exit']['y']
                    elevator_level.exit.theta = level['exit']['theta']
                    elevator.levels.append(elevator_level)
                self.__setitem__(unique_name, elevator)
            except KeyError:
                rospy.logwarn("Elevators : one of the expected fields for elevator '%s' was missing!" % unique_name)

    def __str__(self):
        s = console.bold + console.white + "\nElevators:\n" + console.reset
        for unique_name in sorted(self):
            elevator = dict.__getitem__(self, unique_name)
            s += console.green + "  %s\n" % unique_name
            s += console.cyan + "    name" + console.reset + ": " + console.yellow + "%s\n" % elevator.name
            s += console.cyan + "    levels:\n"
            for level in elevator.levels:
                s += console.cyan + "     - world" + console.reset + ": " + console.yellow + "%s\n" % level.world
                s += console.cyan + "       floor" + console.reset + ": " + console.yellow + "%s\n" % level.floor
                s += console.cyan + "       entry" + console.reset + ": " + console.yellow + "{ x: %s, y: %s, theta: %s }\n" % (level.entry.x, level.entry.y, level.entry.theta)
                s += console.cyan + "       exit " + console.reset + ": " + console.yellow + "{ x: %s, y: %s, theta: %s }\n" % (level.exit.x, level.exit.y, level.exit.theta)
        s += console.reset
        return s

    def find_level_on_elevator(self, elevator_unique_name, world):
        """
        Find the level corresponding to the elevator name and world key.

        :returns: matching elevator level object if found
        :rtype: gopher_semantic_msgs.ElevatorLevel or None
        """
        try:
            elevator = dict.__getitem__(self, elevator_unique_name)
        except KeyError:
            return None
        for level in elevator.levels:
            if level.world == world:
                return level
        return None

    def find_connecting_elevators(self, origin, destination):
        """
        :param str origin: origin world unique name.
        :param str destination: origin world unique name.
        :return: list of elevators that connect origin to destination
        :rtype: gopher_semantic_msgs.Elevator
        """
        connecting_elevators = []
        for elevator in dict.values(self):
            connecting_worlds = [level.world for level in elevator.levels]
            if origin in connecting_worlds and destination in connecting_worlds:
                connecting_elevators.append(elevator)
        return connecting_elevators

    def to_msg(self):
        msg = gopher_semantic_msgs.Elevators()
        msg.elevators = self.values()
        return msg

    def spin(self):
        rospy.spin()


def find_connecting_levels(elevator, origin, destination):
    """
    :param gopher_semantic_msgs.Elevator elevator: elevator to investigate
    :param str origin: unique world name of the origin
    :param str destination:  unique world name of the destination
    :return: the two levels that connect origin to world as a pair (origin_level, world_level)
    :rtype: (gopher_semantic_msgs.ElevatorLevel, gopher_semantic_msgs.ElevatorLevel) or None types when levels aren't found.
    """
    origin_level = None
    destination_level = None
    for level in elevator.levels:
        if origin == level.world:
            origin_level = level
        elif destination == level.world:
            destination_level = level
    return (origin_level, destination_level)

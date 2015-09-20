#
# License: Yujin
#
##############################################################################
# Description
##############################################################################

"""
.. module:: goals
   :platform: Unix
   :synopsis: Goal related functionality.

Oh my spaghettified magnificence,
Bless my noggin with a tickle from your noodly appendages!

----

"""

##############################################################################
# Imports
##############################################################################

from enum import Enum
import gopher_configuration
import gopher_navi_msgs.msg as gopher_navi_msgs
import gopher_semantics
import rospy
import std_msgs.msg as std_msgs

from . import utilities

##############################################################################
# Enums
##############################################################################

# """ An enumerator representing the state of execution """
ExecutionState = Enum('ExecutionState', 'IDLE PAUSED UNPAUSED')

##############################################################################
# Goal Handler
##############################################################################


class GoalHandler(object):
    """
    :ivar result: action result for teleports, gets filled in whenever there is an error.
    :vartype result: gopher_navi_msgs/TeleportResult
    :ivar map_filename: path to an existing map filename to be used in the switching process
    :vartype map_filename: str
    :ivar pose: pose used to initialise on the current map
    :vartype result: geometry_msgs/PoseWithCovarianceStamped
    """
    class Commands(object):
        def __init__(self):
            self.map_filename = None
            self.pose = None

    def __init__(self, publishers, service_proxies):
        """
        :param publishers: publishers from :class:`.Node`, including both init_pose and switch_map publishers.
        :param service_proxies: as above, specifically clear_proxies
        """
        # basics
        self.command = GoalHandler.Commands()
        self.gopher = gopher_configuration.Configuration()
        self.semantics = gopher_semantics.Semantics(self.gopher.namespaces.semantics)
        self.result = gopher_navi_msgs.TeleportResult()
        self.service_proxies = service_proxies
        self.publishers = publishers
        self.execution_state = ExecutionState.IDLE
        self.timer = None

        # note : maps and worlds might be different (might have installed more maps than worlds)
        self.maps = utilities.get_installed_maps()
        self.worlds = utilities.get_installed_worlds(self.maps)

    def clear(self):
        self.command = GoalHandler.Commands()
        self.result = gopher_navi_msgs.TeleportResult()
        self.result.value = gopher_navi_msgs.TeleportResult.SUCCESS
        self.result.message = "success"

    def execute(self):
        """
        Note: this function doesn't return success or otherwise. Simply returns whether the
        execution is done and dusted or not.

        :return: Whether we're done with it or not.
        :rtype: bool
        """
        if self.execution_state == ExecutionState.IDLE:
            # we've just loaded a goal (we trust the node to make sure that this is so!
            if self.command.map_filename is not None:
                # we need to switch maps.
                self.publishers.switch_map.publish(std_msgs.String(self.command.map_filename))
                if self.command.pose is None:
                    return True  # we're done
            if self.command.pose is not None:
                self.publishers.init_pose.publish(self.command.pose)
                self.execution_state = ExecutionState.PAUSED
                # need to give it enough time for the init pose to succeed otherwise
                # we'll clear locally, then sense/save on the costmap and then init pose
                # somewhere else...the original location's saved marks on the costmap will
                # still be there.
                self.timer = rospy.Timer(rospy.Duration(1), self._unpause, oneshot=True)
                return False
            self.result.value = gopher_navi_msgs.TeleportResult.ERROR_INVALID_ARGUMENTS
            self.result.message = "execution has both map and pose command 'None' (shouldn't get here)"
            rospy.logerr("Marco Polo : %s" % self.result.message)
            return True
        elif self.execution_state == ExecutionState.UNPAUSED:
            # Clear the costmaps, but just carry on anyway if it fails as it is not a critical error.
            try:
                self.service_proxies.clear_costmaps()
            except rospy.ServiceException:
                rospy.logwarn("Marco Polo : failed to clear costmaps, wrong service name name? [%s]" % self.gopher.services.clear_costmap)
            except rospy.ROSInterruptException:
                rospy.logwarn("Marco Polo : interrupted while trying to clear costmaps, probably ros shutting down")
            self.execution_state = ExecutionState.IDLE
            return True
        else:
            return False

    def _unpause(self, unused_timer_event):
        self.execution_state = ExecutionState.UNPAUSED

    def load(self, goal):
        """
        The goal can be one of various types as commented upon in the action
        file itself. This parses the goal and sets the apprioriate map_filename
        and pose that will be used for the execution.

        If there is an error, this function returns false and leave the
        appropriate error message and code in the result variable.

        :param gopher_navi_msgs.TeleportGoal goal:
        :returns: true or false depending on success.
        """
        ######################################################################
        # Switch maps
        ######################################################################
        if goal.world:
            if goal.world in self.maps.keys():
                rospy.loginfo("Marco Polo : switching worlds (maps) to '%s'" % goal.world)
                self.command.map_filename = self.maps[goal.world]
                self.command.pose = None
                return True
            else:
                self.result.value = gopher_navi_msgs.TeleportResult.ERROR_MAP_FILE_DOES_NOT_EXIST
                self.result.message = "requested map file not installed [%s]" % goal.world
                return False

        ######################################################################
        # Teleport to a semantic location
        ######################################################################
        if goal.location:
            if goal.location not in self.semantics.locations:
                self.result.value = gopher_navi_msgs.TeleportResult.ERROR_SEMANTIC_LOCATION_DOES_NOT_EXIST
                self.result.message = "requested semantic location does not exist [%s]" % goal.location
                return False
            location = self.semantics.locations[goal.location]
            if location.world not in self.semantics.worlds:
                self.result.value = gopher_navi_msgs.TeleportResult.ERROR_SEMANTIC_WORLD_DOES_NOT_EXIST
                self.result.message = "requested semantic world does not exist (bad semantics database?) [%s]" % location.world
                return False
            if location.world not in self.semantics.worlds:
                self.result.value = gopher_navi_msgs.TeleportResult.ERROR_MAP_FILE_DOES_NOT_EXIST
                self.result.message = "requested semantic world map file does not exist [%s]" % location.world
                return False
            self.command.map_filename = self.worlds[location.world]
            self.command.pose = utilities.msg_pose2d_to_pose_with_covariance_stamped(location.pose, self.gopher.frames.map)
            rospy.loginfo("Marco Polo : teleporting to semantic location [%s][%s][(%s,%s)]" % (goal.location, location.world, self.command.pose.pose.pose.position.x, self.command.pose.pose.pose.position.y))
            return True

        ######################################################################
        # Teleport to an elevator location
        ######################################################################
        if goal.elevator_location.elevator:
            elevator_name = goal.elevator_location.elevator
            if elevator_name not in self.semantics.elevators:
                self.result.value = gopher_navi_msgs.TeleportResult.ERROR_SEMANTIC_LOCATION_DOES_NOT_EXIST
                self.result.message = "requested elevator does not exist [%s]" % elevator_name
                return False
            elevator_level = self.semantics.elevators.find_level_on_elevator(elevator_name, goal.elevator_location.world)
            if elevator_level is None:
                self.result.value = gopher_navi_msgs.TeleportResult.ERROR_SEMANTIC_LOCATION_DOES_NOT_EXIST
                self.result.message = "requested elevator does not connect with that world [%s][%s]" % (elevator_name, goal.elevator_location.world)
                return False
            if elevator_level.world in self.maps.keys():
                self.command.map_filename = self.maps[elevator_level.world]
            else:
                self.result.value = gopher_navi_msgs.TeleportResult.ERROR_MAP_FILE_DOES_NOT_EXIST
                self.result.message = "requested semantic world map file does not exist [%s]" % elevator_level.world
                return False
            pose_2d = elevator_level.entry if goal.elevator_location.location == gopher_navi_msgs.ElevatorLocation.ENTRY else elevator_level.exit
            self.command.pose = utilities.msg_pose2d_to_pose_with_covariance_stamped(pose_2d, self.gopher.frames.map)
            rospy.loginfo("Marco Polo : teleporting to elevator location [%s][%s][(%s,%s)]" % (goal.elevator_location.elevator, goal.elevator_location.world, self.command.pose.pose.pose.position.x, self.command.pose.pose.pose.position.y))
            return True

        ######################################################################
        # Teleport to a world pose pair
        ######################################################################
        # This one is ambiguous so it is last
        if goal.world_pose.world:
            if goal.world_pose.world in self.maps:
                self.command.map_filename = self.maps[goal.world_pose.world]
            else:
                self.result.value = gopher_navi_msgs.TeleportResult.ERROR_MAP_FILE_DOES_NOT_EXIST
                self.result.message = "requested semantic world map file does not exist [%s]" % goal.world_pose.world
                return False
        else:
            self.command.map_filename = None
        self.command.pose = goal.world_pose.pose
        return True

#
# License: Yujin
#
##############################################################################
# Description
##############################################################################

"""
.. module:: node
   :platform: Unix
   :synopsis: Core node api for teleporting/map switching management.

Oh my spaghettified magnificence,
Bless my noggin with a tickle from your noodly appendages!

----

"""

##############################################################################
# Imports
##############################################################################

import actionlib
import diagnostic_msgs.msg as diagnostic_msgs
import geometry_msgs.msg as geometry_msgs
import gopher_configuration
import gopher_semantics
import gopher_std_msgs.msg as gopher_std_msgs
import gopher_navi_msgs.msg as gopher_navi_msgs
import rocon_console.console as console
import rocon_python_comms
import rospy
import std_msgs.msg as std_msgs
import std_srvs.srv as std_srvs
import threading

from . import actions
from . import goals

##############################################################################
# Configuration
##############################################################################


def _error_logger(msg):
    print(console.red + "Configuration : %s" % msg + console.reset)

##############################################################################
# Configuration
##############################################################################


class Node(object):
    def __init__(self):
        self.gopher = gopher_configuration.Configuration()
        self.semantics = gopher_semantics.Semantics(self.gopher.namespaces.semantics)
        self.current_world = None
        self.result = gopher_navi_msgs.TeleportResult()
        self.diagnostics = None
        self.diagnostics_guard = threading.Lock()
        self.action_server = actionlib.SimpleActionServer('teleport',
                                                          gopher_navi_msgs.TeleportAction,
                                                          execute_cb=self.execute,
                                                          auto_start=False
                                                          )
        self.action_server.start()
        self.actions = type('Actions',
                            (object,),
                            {'switch_map': actions.SwitchMap(self.gopher.topics.switch_map),
                             'init_pose': actions.InitPose(self.gopher.topics.initial_pose)
                             }
                            )()
        latched = True
        not_latched = False
        queue_size_five = 5
        self.publishers = rocon_python_comms.utils.Publishers(
            [
                ('~world', std_msgs.String, latched, queue_size_five),
                ('~introspection/maps', std_msgs.String, latched, queue_size_five),
                ('~introspection/worlds', std_msgs.String, latched, queue_size_five),
                ('init_pose', self.gopher.topics.initial_pose, geometry_msgs.PoseWithCovarianceStamped, latched, queue_size_five),
                ('switch_map', self.gopher.topics.switch_map, std_msgs.String, latched, queue_size_five),
                ('diagnostics', self.gopher.topics.diagnostics, diagnostic_msgs.DiagnosticArray, latched, queue_size_five)
                ('teleport_sound', self.gopher.sounds.teleport, std_msgs.Empty, not_latched, 1),
                ('teleport_pattern', self.gopher.topics.display_notification, gopher_std_msgs.Notification, not_latched, 1),
            ]
        )
        self.service_proxies = rocon_python_comms.utils.ServiceProxies(
            [
                (self.gopher.services.clear_costmaps, std_srvs.Empty),
            ]
        )
        self.goal_handler = goals.GoalHandler(self.publishers, self.service_proxies)
        self._publish_introspection_data()

        # look for an initial world from rosparam server
        initial_world = rospy.get_param("~initial_world", None)
        # fallback to the one in the semantics
        if initial_world is None:
            initial_world = self.semantics.worlds.default

        self.publishers.world.publish(initial_world)  # initialise the world publisher

        # look for an initial world from rosparam server
        goal = gopher_navi_msgs.TeleportGoal()
        goal.world = initial_world
        # load it
        if goal.world is not None:
            if self.goal_handler.load(goal):
                # only shifting worlds without an init pose - it's a one shot call
                # if fails, it is only because we're already there and there is a ROS_WARN inside switch_map()
                # TODO : change that to a result, message like loading does.
                successfully_switched_maps = self.goal_handler.switch_map(self.goal_handler.command.map_filename)
                if successfully_switched_maps:
                    self.generate_diagnostics(diagnostic_msgs.DiagnosticStatus.OK, goal.world, "Loaded '%s'." % goal.world, "success")
                else:
                    # Don't do a warning here as it will stay a warning for a long time when the system is actually ok
                    self.generate_diagnostics(diagnostic_msgs.DiagnosticStatus.OK, goal.world, "Loaded '%s'." % goal.world, "found map, but did not switch as we are already there.")
            else:
                rospy.logerr("MarcoPolo : %s" % (self.goal_handler.result.message))
                self.generate_diagnostics(diagnostic_msgs.DiagnosticStatus.ERROR, goal.world, "Failed to initialise with a map [%s]." % goal.world, self.goal_handler.result.message)
        else:
            self.generate_diagnostics(diagnostic_msgs.DiagnosticStatus.ERROR, "none", "Did not load a map.", "no map was specified, either via param or semantics.")

        self.timer = rospy.Timer(rospy.Duration(3), self.publish_diagnostics)

    def publish_diagnostics(self, unused_event):
        self.diagnostics_guard.acquire()
        try:
            self.publishers.diagnostics.publish(self.diagnostics)
        except rospy.ROSException:
            # not init'd or shutting down, either way, not needed to be handled here.
            pass
        self.diagnostics_guard.release()

    def generate_diagnostics(self, level, map_name, message, loading_message):
        '''
        :param byte level: one of the diagnostic_msgs.DiagnosticStatus levels (OK, WARN, ERROR, STALE)
        :param str map_name: name of the map file to load
        :param str message: detailed message about the map loading operation.
        '''
        msg = diagnostic_msgs.DiagnosticStatus()
        msg.name = 'marco_polo::Map Loading'
        msg.level = level
        map_name_diagnostic = diagnostic_msgs.KeyValue()
        map_name_diagnostic.key = 'name'
        map_name_diagnostic.value = map_name
        msg.values.append(map_name_diagnostic)
        loading_result_diagnostic = diagnostic_msgs.KeyValue()
        loading_result_diagnostic.key = 'result'
        loading_result_diagnostic.value = loading_message
        msg.values.append(loading_result_diagnostic)
        msg.message = message
        diagnostic_array_msg = diagnostic_msgs.DiagnosticArray()
        diagnostic_array_msg.header.stamp = rospy.Time.now()
        diagnostic_array_msg.status.append(msg)
        self.diagnostics_guard.acquire()
        self.diagnostics = diagnostic_array_msg
        self.diagnostics_guard.release()

    def execute(self, goal):
        """
        :param gopher_navi_msgs.msg.TeleportGoal goal: incoming action server goal.
        """
        # goal.target_pose = don't care
        frequency = 1
        rate = rospy.Rate(frequency)  # hz
        rospy.loginfo("MarcoPolo : received a goal")
        # if we just received a goal, we erase any previous pre-emption
        self.action_server.preempt_request = False
        self.goal_handler.clear()
        if not self.goal_handler.load(goal):
            self.result = self.goal_handler.result
            self.action_server.set_aborted(self.result, self.result.message)
            rospy.logwarn("MarcoPolo : rejected teleport goal [%s][%s]" % (self.result.value, self.result.message))
            # only one possible cause of failure here - map file could not be found
            self.generate_diagnostics(diagnostic_msgs.DiagnosticStatus.WARN, goal.world, "Requested map loading failed.", self.goal_handler.result.message)
            return
        self.generate_diagnostics(diagnostic_msgs.DiagnosticStatus.OK, goal.world, "Map successfully loaded.", "success")
        self.result.value = gopher_navi_msgs.TeleportResult.SUCCESS
        self.result.message = "success"
#         if goal.special_effects:
#             self.publishers.teleport_sound.publish(std_msgs.Empty())
#             self.publishers.teleport_pattern.publish(gopher_std_msgs.Notification(led_pattern=self.gopher.led_patterns.holding))
        while not rospy.is_shutdown():
            # preempted
            if self.action_server.is_preempt_requested():
                rospy.loginfo("MarcoPolo : preempt requested")
                self.result.value = gopher_navi_msgs.TeleportResult.ERROR_PREEMPTED
                self.result.message = "preempted by another teleport goal request."
                self.action_server.set_preempted(self.result, self.result.message)
                break
            if self.goal_handler.execute():
                rospy.loginfo("MarcoPolo : teleport success")
                self.action_server.set_succeeded(self.result, "successfully teleported!")
                break
            else:
                self.action_server.publish_feedback(gopher_navi_msgs.TeleportFeedback("waiting before clearing costmap"))
                rospy.logdebug("")
            rate.sleep()
        # don't worry about cancelling the led pattern, it will phase out after 10s.

    ##############################################################################
    # Private Methods
    ##############################################################################

    def _publish_introspection_data(self):
        s = console.bold + "\nMaps\n" + console.reset
        for key in sorted(self.goal_handler.maps):
            value = self.goal_handler.maps[key]
            s += console.cyan + "  %s" % key + console.reset + ": " + console.yellow + (value if value is not None else '-') + console.reset + "\n"
        self.publishers.maps.publish(std_msgs.String(s))
        s = console.bold + "\nWorlds\n" + console.reset
        for key in sorted(self.goal_handler.worlds):
            value = self.goal_handler.worlds[key]
            s += console.cyan + "  %s" % key + console.reset + ": " + console.yellow + (value if value is not None else '-') + console.reset + "\n"
        self.publishers.worlds.publish(std_msgs.String(s))

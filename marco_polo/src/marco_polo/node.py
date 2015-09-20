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
import geometry_msgs.msg as geometry_msgs
import gopher_configuration
import gopher_semantics
import gopher_navi_msgs.msg as gopher_navi_msgs
import rocon_console.console as console
import rocon_python_comms
import rospy
import std_msgs.msg as std_msgs
import std_srvs.srv as std_srvs
import transitions

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
        queue_size_five = 5
        self.publishers = rocon_python_comms.utils.Publishers(
            [
                ('~world', std_msgs.String, latched, queue_size_five),
                ('~introspection/maps', std_msgs.String, latched, queue_size_five),
                ('~introspection/worlds', std_msgs.String, latched, queue_size_five),
                ('init_pose', self.gopher.topics.initial_pose, geometry_msgs.PoseWithCovarianceStamped, latched, queue_size_five),
                ('switch_map', self.gopher.topics.switch_map, std_msgs.String, latched, queue_size_five),
            ]
        )
        self.service_proxies = rocon_python_comms.utils.ServiceProxies(
            [
                (self.gopher.services.clear_costmaps, std_srvs.Empty),
            ]
        )
        self.goal_handler = goals.GoalHandler(self.publishers, self.service_proxies)
        self._publish_introspection_data()
        self.publishers.world.publish(std_msgs.String())  # initialise the world publisher

        # look for an initial world from rosparam server
        goal = gopher_navi_msgs.TeleportGoal()
        goal.world = rospy.get_param("~world", None)
        # if nothing there, fallback to the one in the semantics
        if goal.world is None:
            goal.world = self.semantics.worlds.default
        # load it
        if goal.world is not None:
            if self.goal_handler.load(goal):
                # only shifting worlds without an init pose - it's a one shot call
                self.goal_handler.switch_map(self.goal_handler.command.map_filename)

    def execute(self, goal):
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
            return
        self.result.value = gopher_navi_msgs.TeleportResult.SUCCESS
        self.result.message = "success"
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

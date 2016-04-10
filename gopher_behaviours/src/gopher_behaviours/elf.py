#!/usr/bin/env python
#
# License: Yujin
#
##############################################################################
# Documentation
##############################################################################

"""
.. module:: elf
   :platform: Unix
   :synopsis: ELF localisation behaviours

Oh my spaghettified magnificence,
Bless my noggin with a tickle from your noodly appendages!
"""

##############################################################################
# Imports
##############################################################################

import enum
import elf_msgs.msg as elf_msgs
import gopher_configuration
import gopher_std_msgs.msg as gopher_std_msgs
import gopher_std_msgs.srv as gopher_std_srvs
import py_trees
import rospy
import std_msgs.msg as std_msgs

from . import ar_markers
from . import interactions
from . import navigation
from . import simple_motions
from . import transform_utilities

##############################################################################
# Enum
##############################################################################


class ElfInitialisationType(enum.IntEnum):
    """ An enumerator representing the status of a behaviour """

    TELEOP = 0
    """Elf initialisation via teleop/teleport"""
    AR = 1
    """Elf initialiation via ar markers"""


string_to_elf_initialisation_type = {
    "teleop": ElfInitialisationType.TELEOP,
    "ar": ElfInitialisationType.AR
}

##############################################################################
# Behaviours
##############################################################################


def create_localisation_check_behaviour(name="Localised?"):
    """
    Hooks up a subscriber to the elf to check if everything is
    nice and localised.
    """
    gopher = gopher_configuration.Configuration(fallback_to_defaults=True)
    behaviour = py_trees.CheckSubscriberVariable(
        name=name,
        topic_name=gopher.topics.elf_status,
        topic_type=elf_msgs.ElfLocaliserStatus,
        variable_name="status",
        expected_value=elf_msgs.ElfLocaliserStatus.STATUS_WORKING,
        fail_if_no_data=True,
        fail_if_bad_comparison=True,
        monitor_continuously=True
    )
    return behaviour


def create_localisation_to_blackboard_behaviour(
        name="ElfToBlackboard",
        blackboard_variables={"elf_localisation_status", None}
):
    """
    Hooks up a subscriber to the elf and transfers the status
    message to the blackboard.
    """
    gopher = gopher_configuration.Configuration(fallback_to_defaults=True)

    behaviour = py_trees.subscribers.ToBlackboard(
        name=name,
        topic_name=gopher.topics.elf_status,
        topic_type=elf_msgs.ElfLocaliserStatus,
        blackboard_variables=blackboard_variables,
        clearing_policy=py_trees.common.ClearingPolicy.NEVER
    )
    return behaviour


def create_pose_to_blackboard_behaviour(
    name="Elf Pose To Blackboard",
    blackboard_variables={"pose", None}
):
    """
    Hooks up a subscriber and transfers the elf pose to the blackboard.

    :param str name: behaviour name
    :param str blackboard_variable_name: name to write the message to
    :returns: the behaviour
    :rtype: subscribers.CheckSubscriberVariable
    """
    gopher = gopher_configuration.Configuration(fallback_to_defaults=True)

    behaviour = py_trees.subscribers.ToBlackboard(
        name,
        topic_name=gopher.topics.elf_status,
        topic_type=elf_msgs.ElfLocaliserStatus,
        blackboard_variables=blackboard_variables
    )
    return behaviour


##############################################################################
# Subtrees
##############################################################################

def create_check_elf_status_subtree():
    """
    Create the subtree responsible for checking the elf status. It is
    non-blocking until it finds a localised status, at which point it halts
    briefly to let you know (and celebrate!).
    """
    gopher = gopher_configuration.Configuration(fallback_to_defaults=True)
    localised_yet = py_trees.composites.Sequence(name="Localised Elf?")
    localised_yet.blackbox_level = py_trees.common.BlackBoxLevel.DETAIL
    check_elf_status = py_trees.CheckSubscriberVariable(
        name="Check ELF Status",
        topic_name=gopher.topics.elf_status,
        topic_type=elf_msgs.ElfLocaliserStatus,
        variable_name="status",
        expected_value=elf_msgs.ElfLocaliserStatus.STATUS_WORKING,
        fail_if_no_data=True,
        fail_if_bad_comparison=True,
        clearing_policy=py_trees.common.ClearingPolicy.NEVER
    )
    notify_done = interactions.SendNotification(
        "Celebrate",
        led_pattern=gopher_std_msgs.LEDStrip.AROUND_RIGHT_GREEN,
        sound="",
        message="intialised pose"
    )
    take_time_to_celebreate = py_trees.timers.Timer("Take Time to Celebrate", 2.0)
    localised_yet.add_child(check_elf_status)
    localised_yet.add_child(notify_done)
    notify_done.add_child(take_time_to_celebreate)
    return localised_yet

##############################################################################
# ELF Initialisers
##############################################################################


class TeleopInitialisation(py_trees.Selector):
    def __init__(self, name="Teleop Elf"):
        super(TeleopInitialisation, self).__init__(name)
        self.blackbox_level = py_trees.common.BlackBoxLevel.COMPONENT
        self.gopher = gopher_configuration.Configuration(fallback_to_defaults=True)

        localised_yet = create_check_elf_status_subtree()
        initialising = py_trees.composites.Sequence("Teleop & Teleport")
        teleop_to_homebase = interactions.SendNotification(
            name='Teleop to Homebase',
            message='waiting for button press to continue',
            led_pattern=self.gopher.led_patterns.humans_i_need_help,
            duration=gopher_std_srvs.NotifyRequest.INDEFINITE
        )
        wait_for_go_button_press = interactions.create_wait_for_go_button("Wait for Go Button")
        hang_around = py_trees.behaviours.Running("Hang Around")
        teleport = navigation.create_homebase_teleport()

        self.add_child(localised_yet)
        self.add_child(initialising)
        initialising.add_child(teleop_to_homebase)
        teleop_to_homebase.add_child(wait_for_go_button_press)
        initialising.add_child(teleport)
        initialising.add_child(hang_around)

#     def setup(self, timeout):
#         # This publisher 'blips' like a radar while the initialising subtree is scanning.
#         # Not useful on the robot except for debugging, but useful for podium simulations and tests
#         self.fake_init_publisher = rospy.Publisher(self.gopher.topics.elf_fake_init, std_msgs.Empty, queue_size=1)
#         return True
#
#     def update(self):
#         if self.status == py_trees.common.Status.RUNNING:
#             print("Publishing Fake Init")
#             self.fake_init_publisher.publish(std_msgs.Empty())


class ARInitialisation(py_trees.Sequence):
    """
    This class implements the sequence of initialising the robot's pose
    using means from the hint providers in the elf localisation framework.

    Blackboard Variables:

     - auto_init_failed     (w) [bool]      : signal to others that the initialisation failed.
    """
    def __init__(self, name="AR Elf"):
        """
        Put together the pose intialisation sequence
        """
        super(ARInitialisation, self).__init__(name)
        self.gopher = gopher_configuration.Configuration(fallback_to_defaults=True)
        self.blackbox_level = py_trees.common.BlackBoxLevel.COMPONENT

        # Behaviours
        # only long range needed
        ar_markers_on = ar_markers.ControlARMarkerTracker("AR Markers On", self.gopher.topics.ar_tracker_long_range, True)
        ar_markers_off = ar_markers.ControlARMarkerTracker("AR Markers Off", self.gopher.topics.ar_tracker_long_range, False)
        ar_markers_off_two = py_trees.meta.success_is_failure(ar_markers.ControlARMarkerTracker)("AR Markers Off", self.gopher.topics.ar_tracker_long_range, False)

        scanning = py_trees.Selector("Scanning")
        localised_yet = create_check_elf_status_subtree()
        # always succeed so we can go on to the timer
        rotate_ad_nauseum = simple_motions.create_rotate_ad_nauseum(5.0)
        timed_out = py_trees.composites.Sequence("Timed Out")
        write_auto_initialisation_failed = py_trees.blackboard.SetBlackboardVariable("Flag AutoInit Failed", "auto_init_failed", True)

        self.add_child(ar_markers_on)
        self.add_child(scanning)
        scanning.add_child(localised_yet)
        scanning.add_child(rotate_ad_nauseum)
        scanning.add_child(timed_out)
        timed_out.add_child(write_auto_initialisation_failed)
        timed_out.add_child(ar_markers_off_two)  # cleaning up, just make sure we send fail from here
        self.add_child(ar_markers_off)

    def setup(self, timeout):
        # This publisher 'blips' like a radar while the initialising subtree is scanning.
        # Not useful on the robot except for debugging, but useful for podium simulations and tests
        self.fake_init_publisher = rospy.Publisher(self.gopher.topics.elf_fake_init, std_msgs.Empty, queue_size=1)
        return True

    def update(self):
        if self.status == py_trees.common.Status.RUNNING:
            self.fake_init_publisher.publish(std_msgs.Empty())

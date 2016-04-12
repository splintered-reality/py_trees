#
# License: Yujin
#
##############################################################################
# Description
##############################################################################
from billiard.py2.connection import FAILURE

"""
.. module:: interactions
   :platform: Unix
   :synopsis: Behaviours for gopher interactions with humans.

Oh my spaghettified magnificence,
Bless my noggin with a tickle from your noodly appendages!

----

"""

##############################################################################
# Imports
##############################################################################

import gopher_configuration
import gopher_std_msgs.msg as gopher_std_msgs
import gopher_std_msgs.srv as gopher_std_srvs
import py_trees
import rocon_console.console as console
import rospy
import std_msgs.msg as std_msgs
import unique_id

##############################################################################
# Creational patterns
##############################################################################


def create_button_event_handler(name="Button Events"):
    """
    A subtree designed to run *every* tick to check for button events and record
    them on the blackboard.

    Will always be in a RUNNING state so be sure to put this in a parallel
    composite...OR use the running_is_failure  decorator and put it under a
    selector at the highest priority level.

    Blackboard Variables:

     - event_go_button    (w) [bool] : true if at least one press, false otherwise
     - event_stop_button  (w) [bool] : true if at least one press, false otherwise
     - event_abort_button (w) [bool] : true if at least one press, false otherwise
     - event_init_button  (w) [bool] : true if at least one press, false otherwise
    """
    event_handler = py_trees.composites.Sequence(name)
    event_handler.blackbox_level = py_trees.common.BlackBoxLevel.DETAIL

    gopher = gopher_configuration.Configuration(fallback_to_defaults=True)
    go_button_events = MonitorButtonEvents(
        name="Go",
        topic_name=gopher.buttons.go,
        variable_name="event_go_button"
    )
    stop_button_events = MonitorButtonEvents(
        name="Stop",
        topic_name=gopher.buttons.stop,
        variable_name="event_stop_button"
    )
    abort_button_events = MonitorButtonEvents(
        name="Abort",
        topic_name=gopher.buttons.abort,
        variable_name="event_abort_button"
    )
    init_button_events = MonitorButtonEvents(
        name="Init",
        topic_name=gopher.buttons.init,
        variable_name="event_init_button"
    )
    event_handler.add_child(go_button_events)
    event_handler.add_child(stop_button_events)
    event_handler.add_child(abort_button_events)
    event_handler.add_child(init_button_events)
    return event_handler


def create_wait_for_go_button(name="Wait For Go Button"):
    """
    Tune into the go button signal. Will be RUNNING until an event
    is caught. This is useful to block at the start of a sequence.

    :param str name: the behaviour name.
    """
    gopher = gopher_configuration.Configuration(fallback_to_defaults=True)
    behaviour = py_trees.subscribers.WaitForSubscriberData(
        name=name,
        topic_name=gopher.buttons.go,
        topic_type=std_msgs.Empty,
        clearing_policy=py_trees.common.ClearingPolicy.ON_INITIALISE
    )
    return behaviour


def create_check_for_go_button_press(name="Check for Go Button Press"):
    """
    Has there recently been a go button press anytime in the recent past?
    This is more useful as a non-blocking decision element (as opposed to
    the :py:func:`~gopher_behaviours.interactions.create_wait_for_go_button`
    function.

    Note that it will reset as soon as an event (i.e. SUCCESS) is detected.

    :param str name: the behaviour name.
    """
    gopher = gopher_configuration.Configuration(fallback_to_defaults=True)
    behaviour = py_trees.meta.running_is_failure(
        py_trees.subscribers.WaitForSubscriberData(
            name=name,
            topic_name=gopher.buttons.go,
            topic_type=std_msgs.Empty,
            clearing_policy=py_trees.common.ClearingPolicy.ON_SUCCESS
        )
    )
    return behaviour


def create_check_for_stop_button_press(name="Check for Stop Button Press"):
    """
    Has there recently been a go button press anytime in the recent past?
    This is more useful as a non-blocking decision element (as opposed to
    the :py:func:`~gopher_behaviours.interactions.create_wait_for_go_button`
    function.

    Note that it will reset as soon as an event (i.e. SUCCESS) is detected.

    :param str name: the behaviour name.
    """
    gopher = gopher_configuration.Configuration(fallback_to_defaults=True)
    MetaBehaviour = py_trees.meta.running_is_failure(py_trees.subscribers.WaitForSubscriberData)
    behaviour = MetaBehaviour(
        name=name,
        topic_name=gopher.buttons.stop,
        topic_type=std_msgs.Empty,
        clearing_policy=py_trees.common.ClearingPolicy.ON_SUCCESS
    )
    return behaviour

##############################################################################
# Behaviours
##############################################################################


class MonitorButtonEvents(py_trees.subscribers.SubscriberHandler):
    """
    This will catch a button event between every tick and write the result to
    the blackboard (True for at least one button event, False otherwise).
    Ideally you need this at the very highest part of the tree so that it
    gets triggered every time - once this happens, then the rest of the behaviour
    tree can utilise the variables.
    """
    def __init__(self,
                 name="Monitor Button Events",
                 topic_name="/gopher/buttons/go",
                 variable_name="go_button_event"
                 ):
        super(MonitorButtonEvents, self).__init__(
            name=name,
            topic_name=topic_name,
            topic_type=std_msgs.Empty,
            clearing_policy=py_trees.common.ClearingPolicy.ON_SUCCESS
        )
        self.variable_name = variable_name
        self.blackboard = py_trees.Blackboard()

    def update(self):
        """
        Check for data and write to the board. We also always return success from
        this as its just a worker, not a logic element.
        """
        self.logger.debug("  %s [MonitorButtonEvents::update()]" % self.name)
        with self.data_guard:
            self.blackboard.set(self.variable_name, self.msg is not None, overwrite=True)
            # ON_SUCCESS is the only clearing_policy that subclasses of SubscriberHandler must implement themselves
            self.msg = None
        return py_trees.common.Status.SUCCESS


class Articulate(py_trees.Behaviour):
    """
    Articulate a sound.
    """
    def __init__(self, name, topic_name="/gopher/commands/sounds/yawn", volume=100):
        """
        He is a mere noodly appendage - don't expect him to check if the topic exists.

        A pastafarian at a higher level should take care of that before construction.

        :param str name: behaviour name
        """
        super(Articulate, self).__init__(name)
        self.publisher = rospy.Publisher(topic_name, std_msgs.Empty, queue_size=1)

    def update(self):
        self.publisher.publish(std_msgs.Empty())
        return py_trees.Status.SUCCESS


class Notification(py_trees.Behaviour):
    """
    This class is ideally run as part of a parallels composite - you want to keep this running while
    other jobs are running and turn it off as soon as one completes. The parallels composite should
    use the SUCCESS_ON_ONE policy.

    A simple example would be to run a WaitForButton next to this one.
    """
    def __init__(self, name,
                 message,
                 sound="",
                 led_pattern=None,
                 button_cancel=gopher_std_msgs.Notification.RETAIN_PREVIOUS,
                 button_confirm=gopher_std_msgs.Notification.RETAIN_PREVIOUS,
                 cancel_on_stop=True,
                 duration=gopher_std_srvs.NotifyRequest.INDEFINITE
                 ):
        """
        :param str name: behaviour name
        :param str led_pattern: any one of the led pattern constants from gopher_std_msgs/Notification
        :param str message: a message for the status notifier to display.
        :param str sound: usually prefer the :py:class:`~gopher_beahviours.interactions.Articulate` behaviour (until we have aliases working inside the status notifier)
        :param str led_pattern:
        :param str button_cancel:
        :param str button_confirm:
        :param bool cancel_on_stop: if true, stop the notification when the behaviour finishes, otherwise display until the notification timeout
        :param int duration: time in seconds for which to display this notification. Default is to display indefinitely. Useful to use with cancel_on_stop set to false.
        """
        super(Notification, self).__init__(name)
        self.gopher = gopher_configuration.Configuration()
        self.topic_name = self.gopher.services.notification
        self.service = rospy.ServiceProxy(self.topic_name, gopher_std_srvs.Notify)
        self.timer = None
        self.sound = sound
        self.service_failed = False
        self.message = message

        led_pattern_id = led_pattern if led_pattern is not None else gopher_std_msgs.Notification.RETAIN_PREVIOUS
        self.led_pattern = gopher_std_msgs.LEDStrip(led_strip_pattern=led_pattern_id)
        self.button_cancel = button_cancel
        self.button_confirm = button_confirm
        self.duration = duration

        self.notification = gopher_std_msgs.Notification(
            sound_name=self.sound,
            led_pattern=self.led_pattern,
            button_confirm=self.button_confirm,
            button_cancel=self.button_cancel,
            override_previous=True,  # DJS is this going to cause problems?
            message=self.message
        )

        self.cancel_on_stop = cancel_on_stop
        # flag used to remember that we have a notification that needs cleaning up or not
        self.sent_notification = False

    def setup(self, timeout):
        self.logger.debug("  %s [Notification::setup()]" % self.name)
        self.logger.info("  %s [Notification::setup()]" % self.name)
        try:
            rospy.wait_for_service(self.topic_name, timeout)
            return True
        except rospy.ROSException:  # timeout
            return False
        except rospy.ROSInterruptException:  # ros shutdown
            return False

    def initialise(self):
        self.logger.debug("  %s [Notification::initialise()]" % self.name)
        self.logger.info("  %s [Notification::initialise()]" % self.name)
        request = gopher_std_srvs.NotifyRequest()
        request.id = unique_id.toMsg(self.id)
        request.action = gopher_std_srvs.NotifyRequest.START
        request.duration = self.duration
        request.notification = self.notification
        try:
            unused_response = self.service(request)
            self.sent_notification = True
        except rospy.ServiceException as e:
            rospy.logwarn("Behaviour [%s" % self.name + "]: failed to process notification request [%s][%s]" % (self.message, str(e)))
            self.service_failed = True
            self.stop(py_trees.Status.FAILURE)
        except rospy.exceptions.ROSInterruptException:
            # ros shutdown, close quietly
            return

    def update(self):
        """
        Not in control of his own actions...must be terminated by a composite above!

        Might be useful having a timer here if the notification is a timed one so that it returns
        success when the notification is supposed to have stopped.
        """
        self.logger.info("  %s [Notification::update()]" % self.name)
        return py_trees.Status.RUNNING

    def terminate(self, new_status):
        """
        Send a stop message to the status notifier if necessary.
        """
        self.logger.debug("  %s [Notification::terminate()][%s->%s]" % (self.name, self.status, new_status))
        self.logger.info("  %s [Notification::terminate()][%s->%s]" % (self.name, self.status, new_status))
        if self.cancel_on_stop and not self.service_failed and self.sent_notification:
            request = gopher_std_srvs.NotifyRequest()
            request.id = unique_id.toMsg(self.id)
            request.action = gopher_std_srvs.NotifyRequest.STOP
            request.notification = gopher_std_msgs.Notification(message=self.message)
            self.sent_notification = False
            try:
                unused_response = self.service(request)
            except rospy.ServiceException as e:
                rospy.logwarn("Behaviour [%s" % self.name + "]: failed to process notification cancel request [%s][%s]" % (self.message, str(e)))
            except rospy.exceptions.ROSInterruptException:
                # ros shutting down
                pass

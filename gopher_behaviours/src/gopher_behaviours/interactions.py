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


def create_wait_for_go_button(name="Wait For Go Button"):
    """
    Tune into the go button signal.

    :param str name: the behaviour name.
    """
    gopher = gopher_configuration.Configuration(fallback_to_defaults=True)
    behaviour = py_trees.subscribers.WaitForSubscriberData(
        name="Wait for Go Button",
        topic_name=gopher.buttons.go,
        topic_type=std_msgs.Empty
    )
    return behaviour

##############################################################################
# Behaviours
##############################################################################


class Articulate(py_trees.Behaviour):
    """
    Articulate a sound.
    """
    def __init__(self, name, topic_name, volume=100):
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


class CheckButtonPressed(py_trees.Behaviour):
    """Checks whether a button has been pressed.

    This behaviour always returns ``SUCCESS`` or ``FAILURE`` (never ``RUNNING``).
    This design is intended to be utilised a guard for a selector.

    Latched is a special characteristic. If latched, it will return true
    and continue returning true if the button is pressed anytime after
    it is 'ticked' for the first time.

    If not latched, it will return the result of a button press
    inbetween ticks.

    :param bool latched: configures the behaviour as described above.
    """
    def __init__(self, name, topic_name, latched=False):
        super(CheckButtonPressed, self).__init__(name)
        self.topic_name = topic_name
        self.subscriber = None
        self.latched = latched
        self.button_pressed = False

    def initialise(self):
        if self.subscriber is None:
            self.subscriber = rospy.Subscriber(self.topic_name, std_msgs.Empty, self.button_callback)

    def button_callback(self, msg):
        self.button_pressed = True

    def update(self):
        if self.button_pressed:
            result = py_trees.Status.SUCCESS
        else:
            result = py_trees.Status.FAILURE
        if not self.latched:
            self.button_pressed = False
        return result


class SendNotification(py_trees.Sequence):
    """
    This class runs as a sequence. Since led's turn off, you need a behaviour that is continuously
    ticking while behaviours underneath are checking for some state to make the whole thing valid.
    In many situations, this does not need to be a full sequence - often it will only be a single
    child.

    A simple example would be to run a WaitForButton or WaitForCharging behaviour beneath this one.

    If not hooked up to the display notications, it will log an error, but quietly 'work' without
    displaying LEDs.
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
        This behaviour is a mere noodly appendage - don't expect it to check if the topic exists for receiving
        the notifications. A pastafarian at a higher level should take care of that before construction.

        :param str name: behaviour name
        :param str led_pattern: any one of the led pattern constants from gopher_std_msgs/Notification
        :param str message: a message for the status notifier to display.
        :param str led_pattern:
        :param str button_cancel:
        :param str button_confirm:
        :param bool cancel_on_stop: if true, stop the notification when the behaviour finishes, otherwise display until the notification timeout
        :param int duration: time in seconds for which to display this notification. Default is to display indefinitely. Useful to use with cancel_on_stop set to false.
        """
        super(SendNotification, self).__init__(name)
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
        self.logger.debug("  %s [SendNotification::setup()]" % self.name)
        # Delayed setup checks, can be done by something like the tree container
        # First the kids
        if not py_trees.Sequence.setup(self, timeout):
            return False
        # Then ourselves
        try:
            rospy.wait_for_service(self.topic_name, timeout)
            return True
        except rospy.ROSException:  # timeout
            return False
        except rospy.ROSInterruptException:  # ros shutdown
            return False

    def initialise(self):
        self.logger.debug("  %s [SendNotification::initialise()]" % self.name)
        request = gopher_std_srvs.NotifyRequest()
        request.id = unique_id.toMsg(self.id)
        request.action = gopher_std_srvs.NotifyRequest.START
        request.duration = self.duration
        request.notification = self.notification
        try:
            unused_response = self.service(request)
            self.sent_notification = True
        except rospy.ServiceException as e:
            rospy.logwarn("SendNotification : failed to process notification request [%s][%s]" % (self.message, str(e)))
            self.service_failed = True
            self.stop(py_trees.Status.FAILURE)
        except rospy.exceptions.ROSInterruptException:
            # ros shutdown, close quietly
            return

    def terminate(self, new_status):
        self.logger.debug("  %s [SendNotification::terminate()][%s->%s]" % (self.name, self.status, new_status))
        if self.cancel_on_stop and not self.service_failed and self.sent_notification:
            request = gopher_std_srvs.NotifyRequest()
            request.id = unique_id.toMsg(self.id)
            request.action = gopher_std_srvs.NotifyRequest.STOP
            request.notification = gopher_std_msgs.Notification(message=self.message)
            self.sent_notification = False
            try:
                unused_response = self.service(request)
            except rospy.ServiceException as e:
                rospy.logwarn("SendNotification : failed to process notification cancel request [%s][%s]" % (self.message, str(e)))
            except rospy.exceptions.ROSInterruptException:
                # ros shutting down
                pass

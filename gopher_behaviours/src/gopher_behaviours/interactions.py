#
# License: Yujin
#
##############################################################################
# Description
##############################################################################

"""
.. module:: interactions
   :platform: Unix
   :synopsis: Behaviours for gopher interactions.

Oh my spaghettified magnificence,
Bless my noggin with a tickle from your noodly appendages!

----

"""

##############################################################################
# Imports
##############################################################################

import gopher_configuration
from gopher_std_msgs.msg import Notification
import py_trees
import rospy
import std_msgs.msg as std_msgs

##############################################################################
# Interactions
##############################################################################


class Articulate(py_trees.Behaviour):
    """
    Articulate a sound.
    """
    def __init__(self, name, topic_name):
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

    This behaviour always returns success or failure. This design is
    intended to be utilised a guard for a selector.

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


class WaitForButton(py_trees.Behaviour):
    def __init__(self, name, topic_name):
        """
        He is a mere noodly appendage - don't expect him to check if the topic exists.

        A pastafarian at a higher level should take care of that before construction.

        :param str name: behaviour name
        :param str topic_name:
        """
        super(WaitForButton, self).__init__(name)
        self.topic_name = topic_name
        self.subscriber = None

    def initialise(self):
        self.subscriber = rospy.Subscriber(self.topic_name, std_msgs.Empty, self.button_callback)
        self.button_pressed = False

    def button_callback(self, msg):
        self.button_pressed = True

    def update(self):
        if self.button_pressed:
            return py_trees.Status.SUCCESS
        else:
            return py_trees.Status.RUNNING

    def stop(self, new_status):
        if self.subscriber is not None:
            self.subscriber.unregister()


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
    def __init__(self, name, message, sound="", led_pattern=None, button_cancel=None, button_confirm=None, cancel_on_stop=True):
        """
        He is a mere noodly appendage - don't expect him to check if the topic exists.

        A pastafarian at a higher level should take care of that before construction.

        :param str name: behaviour name
        :param str led_pattern: any one of the string constants from Notification
        :param str message: a message for the status notifier to display.
        :param bool stop_on_finish: if true, stop the notification when the behaviour finishes, otherwise display until the notification timeout
        """
        super(SendNotification, self).__init__(name)
        self.gopher = gopher_configuration.Configuration()
        self.topic_name = self.gopher.topics.display_notification
        self.publisher = rospy.Publisher(self.topic_name, Notification, queue_size=1)
        self.timer = None
        self.sound = sound
        self.led_pattern = led_pattern if led_pattern is not None else Notification.RETAIN_PREVIOUS
        self.button_cancel = button_cancel if button_cancel is not None else Notification.RETAIN_PREVIOUS
        self.button_confirm = button_confirm if button_confirm is not None else Notification.RETAIN_PREVIOUS

        # cancel LED status on success, but only if they were set by the requested notification
        self.led_stop = Notification.CANCEL_CURRENT if led_pattern is not None else Notification.RETAIN_PREVIOUS
        self.cancel_stop = Notification.CANCEL_CURRENT if button_cancel is not None else Notification.RETAIN_PREVIOUS
        self.confirm_stop = Notification.CANCEL_CURRENT if button_confirm is not None else Notification.RETAIN_PREVIOUS

        self.message = message
        self.cancel_on_stop = cancel_on_stop

    def initialise(self):
        self.send_notification(None)
        # Notifications last for some amount of time before LEDs go back to
        # displaying the battery status. Need to send message repeatedly until
        # the behaviour completes.
        self.timer = rospy.Timer(rospy.Duration(5), self.send_notification)
        super(SendNotification, self).initialise()

    def send_notification(self, unused_timer):
        self.publisher.publish(Notification(sound_name=self.sound, led_pattern=self.led_pattern,
                                            button_confirm=self.button_confirm,
                                            button_cancel=self.button_cancel, message=self.message))

    def stop(self, new_status):
        super(SendNotification, self).stop(new_status)
        if self.timer:
            self.timer.shutdown()
            if self.cancel_on_stop:
                self.publisher.publish(Notification(led_pattern=self.led_stop,
                                                    button_cancel=self.cancel_stop,
                                                    button_confirm=self.confirm_stop))

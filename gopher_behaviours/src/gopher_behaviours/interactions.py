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
import gopher_std_msgs.msg as gopher_std_msgs
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

    def abort(self, new_status):
        if self.subscriber is not None:
            self.subscriber.unregister()


class FlashLEDs(py_trees.Sequence):
    """
    This class runs as a sequence. Since led's turn off, you need a behaviour that is continuously
    ticking while behaviours underneath are checking for some state to make the whole thing valid.
    In many situations, this does not need to be a full sequence - often it will only be a single
    child.

    A simple example would be to run a WaitForButton or WaitForCharging behaviour beneath this one.

    If not hooked up to the display notications, it will log an error, but quietly 'work' without
    displaying LEDs.
    """
    def __init__(self, name, led_pattern, message=""):
        """
        He is a mere noodly appendage - don't expect him to check if the topic exists.

        A pastafarian at a higher level should take care of that before construction.

        :param str name: behaviour name
        :param str led_pattern: any one of the string constants from gopher_std_msgs.Notification
        :param str message: a message for the status notifier to display.
        """
        super(FlashLEDs, self).__init__(name)
        self.gopher = gopher_configuration.Configuration()
        self.topic_name = self.gopher.topics.display_notification
        self.publisher = rospy.Publisher(self.topic_name, gopher_std_msgs.Notification, queue_size=1)
        self.timer = None
        self.led_pattern = led_pattern
        self.message = message

    def initialise(self):
        self.send_notification(None)
        # Notifications last for some amount of time before LEDs go back to
        # displaying the battery status. Need to send message repeatedly until
        # the behaviour completes.
        self.timer = rospy.Timer(rospy.Duration(5), self.send_notification)
        super(FlashLEDs, self).initialise()

    def send_notification(self, unused_timer):
        self.publisher.publish(gopher_std_msgs.Notification(led_pattern=self.led_pattern, message=self.message))

    def abort(self, new_status):
        super(FlashLEDs, self).abort(new_status)
        if self.timer:
            self.timer.shutdown()
            self.publisher.publish(gopher_std_msgs.Notification(led_pattern=gopher_std_msgs.Notification.CANCEL_CURRENT))

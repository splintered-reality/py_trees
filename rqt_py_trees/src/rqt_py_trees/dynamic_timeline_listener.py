#!/usr/bin/env python

import rospy
from .plugins.message_view import MessageView

class DynamicTimelineListener(MessageView):
    """Basic listener for the timeline. The current message can be accessed using
    the ``msg`` member.

    """
    def __init__(self, timeline, topic, signal_viewed, signal_cleared):
        """
        :param timeline: the timeline that this object is attached to
        :param topic: the topic that this object is interested in
        :param signal_viewed: the signal that should be emitted when the message being viewed changes
        :param signal_cleared: the signal that should be emitted when the message being viewed is cleared
        """
        super(DynamicTimelineListener, self).__init__(timeline, topic)
        self.signal_viewed = signal_viewed
        self.signal_cleared = signal_cleared
        self.msg = None

    def message_viewed(self, msg_details):
        """Called whenever the message is updated. Updates the stored message and emits
        a signal.
        """
        self.msg = msg_details
        # rospy.loginfo("got message {0} on topic {1}".format(msg_details.behaviours[0].name, self.topic))
        self.signal_viewed.emit()

    def message_cleared(self):
        self.signal_cleared.emit()

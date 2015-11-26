# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
from .message_view import MessageView

from python_qt_binding.QtGui import QAction, QIcon, QToolBar


class TopicMessageView(MessageView):
    """
    A message view with a toolbar for navigating messages in a single topic.
    """
    def __init__(self, timeline, parent, topic):
        MessageView.__init__(self, timeline, topic)

        self._parent = parent
        self._stamp = None
        self._name = parent.objectName()

        self.toolbar = QToolBar()
        self._first_action = QAction(QIcon.fromTheme('go-first'), '', self.toolbar)
        self._first_action.triggered.connect(self.navigate_first)
        self.toolbar.addAction(self._first_action)
        self._prev_action = QAction(QIcon.fromTheme('go-previous'), '', self.toolbar)
        self._prev_action.triggered.connect(self.navigate_previous)
        self.toolbar.addAction(self._prev_action)
        self._next_action = QAction(QIcon.fromTheme('go-next'), '', self.toolbar)
        self._next_action.triggered.connect(self.navigate_next)
        self.toolbar.addAction(self._next_action)
        self._last_action = QAction(QIcon.fromTheme('go-last'), '', self.toolbar)
        self._last_action.triggered.connect(self.navigate_last)
        self.toolbar.addAction(self._last_action)
        parent.layout().addWidget(self.toolbar)

    @property
    def parent(self):
        return self._parent

    @property
    def stamp(self):
        return self._stamp

    # MessageView implementation

    def message_viewed(self, bag, msg_details):
        _, _, self._stamp = msg_details[:3]

    # Events
    def navigate_first(self):
        for entry in self.timeline.get_entries([self.topic], *self.timeline._timeline_frame.play_region):
            self.timeline._timeline_frame.playhead = entry.time
            break

    def navigate_previous(self):
        last_entry = None
        for entry in self.timeline.get_entries([self.topic], self.timeline._timeline_frame.start_stamp, self.timeline._timeline_frame.playhead):
            if entry.time < self.timeline._timeline_frame.playhead:
                last_entry = entry

        if last_entry:
            self.timeline._timeline_frame.playhead = last_entry.time

    def navigate_next(self):
        for entry in self.timeline.get_entries([self.topic], self.timeline._timeline_frame.playhead, self.timeline._timeline_frame.end_stamp):
            if entry.time > self.timeline._timeline_frame.playhead:
                self.timeline._timeline_frame.playhead = entry.time
                break

    def navigate_last(self):
        last_entry = None
        for entry in self.timeline.get_entries([self.topic], *self.timeline._timeline_frame.play_region):
            last_entry = entry

        if last_entry:
            self.timeline._timeline_frame.playhead = last_entry.time

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

import threading


class MessageLoaderThread(threading.Thread):
    """
    Waits for a new playhead position on the given topic, then loads the message at that position and notifies the view threads.

    One thread per topic.  Maintains a cache of recently loaded messages.
    """
    def __init__(self, timeline, topic):
        threading.Thread.__init__(self)

        self.timeline = timeline
        self.topic = topic

        self.bag_playhead_position = None

        self._message_cache_capacity = 50
        self._message_cache = {}
        self._message_cache_keys = []

        self._stop_flag = False

        self.setDaemon(True)
        self.start()

    def reset(self):
        self.bag_playhead_position = None

    def run(self):
        while not self._stop_flag:
            # Wait for a new entry
            cv = self.timeline._playhead_positions_cvs[self.topic]
            with cv:
                while (self.topic not in self.timeline._playhead_positions) or (self.bag_playhead_position == self.timeline._playhead_positions[self.topic]):
                    cv.wait()
                    if self._stop_flag:
                        return
                bag, playhead_position = self.timeline._playhead_positions[self.topic]

            self.bag_playhead_position = (bag, playhead_position)

            # Don't bother loading the message if there are no listeners
            if not self.timeline.has_listeners(self.topic):
                continue

            # Load the message
            if playhead_position is None:
                msg_data = None
            else:
                msg_data = self._get_message(bag, playhead_position)

            # Inform the views
            messages_cv = self.timeline._messages_cvs[self.topic]
            with messages_cv:
                self.timeline._messages[self.topic] = (bag, msg_data)
                messages_cv.notify_all()      # notify all views that a message is loaded

    def _get_message(self, bag, position):
        key = '%s%s' % (bag.filename, str(position))
        if key in self._message_cache:
            return self._message_cache[key]

        msg_data = self.timeline.read_message(bag, position)

        self._message_cache[key] = msg_data
        self._message_cache_keys.append(key)

        if len(self._message_cache) > self._message_cache_capacity:
            oldest_key = self._message_cache_keys[0]
            del self._message_cache[oldest_key]
            self._message_cache_keys.remove(oldest_key)

        return msg_data

    def stop(self):
        self._stop_flag = True
        cv = self.timeline._playhead_positions_cvs[self.topic]
        with cv:
            cv.notify_all()

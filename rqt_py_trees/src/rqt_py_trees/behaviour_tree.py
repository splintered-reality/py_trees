# Software License Agreement (BSD License)
#
# Copyright (c) 2011, Willow Garage, Inc.
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

from __future__ import division
import os


import rospy
import rospkg
import rosbag
import functools
import py_trees_msgs.msg as py_trees_msgs

from python_qt_binding import loadUi
from python_qt_binding.QtCore import QFile, QIODevice, QObject, Qt, Signal, QEvent
from python_qt_binding.QtGui import QFileDialog, QGraphicsView, QGraphicsScene, QIcon, QImage, QPainter, QWidget, QShortcut, QKeySequence
from python_qt_binding.QtSvg import QSvgGenerator
from qt_dotgraph.pydotfactory import PydotFactory
from qt_dotgraph.dot_to_qt import DotToQtGenerator
from rqt_graph.interactive_graphics_view import InteractiveGraphicsView
from rqt_bag.bag_timeline import BagTimeline
from rqt_bag.bag_widget import BagGraphicsView
from .dynamic_timeline import DynamicTimeline

from .dotcode_behaviour import RosBehaviourTreeDotcodeGenerator
from .timeline_listener import TimelineListener
from .dynamic_timeline_listener import DynamicTimelineListener


class RosBehaviourTree(QObject):

    _deferred_fit_in_view = Signal()
    _refresh_view = Signal()
    _refresh_combo = Signal()
    _message_changed = Signal()
    _message_cleared = Signal()
    _expected_type = py_trees_msgs.BehaviourTree()._type
    _empty_topic = "No valid topics available"
    _unselected_topic = "Not subscribing"

    class ComboBoxEventFilter(QObject):
        """Event filter for the combo box. Will filter left mouse button presses,
        calling a signal when they happen

        """
        def __init__(self, signal):
            """

            :param Signal signal: signal that is emitted when a left mouse button press happens
            """
            super(RosBehaviourTree.ComboBoxEventFilter, self).__init__()
            self.signal = signal

        def eventFilter(self, obj, event):
            if event.type() == QEvent.MouseButtonPress and event.button() == Qt.LeftButton:
                self.signal.emit()
            return False

    def __init__(self, context):
        super(RosBehaviourTree, self).__init__(context)
        self.setObjectName('RosBehaviourTree')

        self.initialized = False
        self._current_dotcode = None # dotcode for the tree that is currently displayed
        self._viewing_bag = False # true if a bag file is loaded
        # True if next or previous buttons are pressed. Reset if the tree being
        # viewed is the last one in the list.
        self._browsing_timeline = False
        # number of messages arrived on the topic since the user started browing the timeline
        self._new_messages = 0
        self.message_list = [py_trees_msgs.BehaviourTree()] # list of all the messages received so far.
        self.current_message = 0 # index of the currently viewed message in the list. 

        self._widget = QWidget()

        # factory builds generic dotcode items
        self.dotcode_factory = PydotFactory()
        # self.dotcode_factory = PygraphvizFactory()
        # generator builds rosgraph
        self.dotcode_generator = RosBehaviourTreeDotcodeGenerator()
        self.current_topic = None
        self.behaviour_sub = None

        # dot_to_qt transforms into Qt elements using dot layout
        self.dot_to_qt = DotToQtGenerator()

        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('rqt_py_trees'), 'resource', 'RosBehaviourTree.ui')
        loadUi(ui_file, self._widget, {'InteractiveGraphicsView': InteractiveGraphicsView})
        self._widget.setObjectName('RosBehaviourTreeUi')
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        self._scene = QGraphicsScene()
        self._scene.setBackgroundBrush(Qt.white)
        self._widget.graphics_view.setScene(self._scene)

        self._widget.highlight_connections_check_box.toggled.connect(self._redraw_graph_view)
        self._widget.auto_fit_graph_check_box.toggled.connect(self._redraw_graph_view)
        self._widget.fit_in_view_push_button.setIcon(QIcon.fromTheme('zoom-original'))
        self._widget.fit_in_view_push_button.pressed.connect(self._fit_in_view)

        self._widget.load_bag_push_button.setIcon(QIcon.fromTheme('document-open'))
        self._widget.load_bag_push_button.pressed.connect(self._load_bag)
        self._widget.load_dot_push_button.setIcon(QIcon.fromTheme('document-open'))
        self._widget.load_dot_push_button.pressed.connect(self._load_dot)
        self._widget.save_dot_push_button.setIcon(QIcon.fromTheme('document-save-as'))
        self._widget.save_dot_push_button.pressed.connect(self._save_dot)
        self._widget.save_as_svg_push_button.setIcon(QIcon.fromTheme('document-save-as'))
        self._widget.save_as_svg_push_button.pressed.connect(self._save_svg)
        self._widget.save_as_image_push_button.setIcon(QIcon.fromTheme('image'))
        self._widget.save_as_image_push_button.pressed.connect(self._save_image)

        # set up the function that is called whenever the box is resized -
        # ensures that the timeline is correctly drawn.
        self._widget.resizeEvent = self._resize_event

        self._timeline = None
        self._timeline_listener = None

        # Connect the message changed function of this object to a corresponding
        # signal. This signal will be activated whenever the message being
        # viewed changes.
        self._message_changed.connect(self.message_changed)
        self._message_cleared.connect(self.message_cleared)

        # Set up combo box for topic selection
        # when the refresh_combo signal happens, update the combo topics available
        self._refresh_combo.connect(self._update_combo_topics)
        # filter events to catch the event which opens the combo box
        self._combo_event_filter = RosBehaviourTree.ComboBoxEventFilter(self._refresh_combo)
        self._widget.topic_combo_box.installEventFilter(self._combo_event_filter)
        self._widget.topic_combo_box.activated.connect(self._choose_topic)
        self._update_combo_topics()

        # Set up navigation buttons
        self._widget.previous_tool_button.pressed.connect(self._previous)
        self._widget.previous_tool_button.setIcon(QIcon.fromTheme('go-previous'))
        self._widget.next_tool_button.pressed.connect(self._next)
        self._widget.next_tool_button.setIcon(QIcon.fromTheme('go-next'))
        self._widget.first_tool_button.pressed.connect(self._first)
        self._widget.first_tool_button.setIcon(QIcon.fromTheme('go-first'))
        self._widget.last_tool_button.pressed.connect(self._last)
        self._widget.last_tool_button.setIcon(QIcon.fromTheme('go-last'))

        # play, pause and stop buttons
        self._widget.play_tool_button.pressed.connect(self._play)
        self._widget.play_tool_button.setIcon(QIcon.fromTheme('media-playback-start'))
        self._widget.stop_tool_button.pressed.connect(self._stop)
        self._widget.stop_tool_button.setIcon(QIcon.fromTheme('media-playback-stop'))
        # also connect the navigation buttons so that they stop the timer when
        # pressed while the tree is playing.
        self._widget.first_tool_button.pressed.connect(self._stop)
        self._widget.previous_tool_button.pressed.connect(self._stop)
        self._widget.last_tool_button.pressed.connect(self._stop)
        self._widget.next_tool_button.pressed.connect(self._stop)

        # set up shortcuts for navigation (vim)
        next_shortcut_vi = QShortcut(QKeySequence("l"), self._widget)
        next_shortcut_vi.activated.connect(self._widget.next_tool_button.pressed)
        previous_shortcut_vi = QShortcut(QKeySequence("h"), self._widget)
        previous_shortcut_vi.activated.connect(self._widget.previous_tool_button.pressed)
        first_shortcut_vi = QShortcut(QKeySequence("^"), self._widget)
        first_shortcut_vi.activated.connect(self._widget.first_tool_button.pressed)
        last_shortcut_vi = QShortcut(QKeySequence("$"), self._widget)
        last_shortcut_vi.activated.connect(self._widget.last_tool_button.pressed)

        # shortcuts for emacs
        next_shortcut_emacs = QShortcut(QKeySequence("Ctrl+f"), self._widget)
        next_shortcut_emacs.activated.connect(self._widget.next_tool_button.pressed)
        previous_shortcut_emacs = QShortcut(QKeySequence("Ctrl+b"), self._widget)
        previous_shortcut_emacs.activated.connect(self._widget.previous_tool_button.pressed)
        first_shortcut_emacs = QShortcut(QKeySequence("Ctrl+a"), self._widget)
        first_shortcut_emacs.activated.connect(self._widget.first_tool_button.pressed)
        last_shortcut_emacs = QShortcut(QKeySequence("Ctrl+e"), self._widget)
        last_shortcut_emacs.activated.connect(self._widget.last_tool_button.pressed)

        # set up stuff for dotcode cache
        self._dotcode_cache_capacity = 50
        self._dotcode_cache = {}
        # cache is ordered on timestamps from messages, but earliest timestamp
        # isn't necessarily the message that was viewed the longest time ago, so
        # need to store keys
        self._dotcode_cache_keys = []

        # set up stuff for scene cache (dotcode cache doesn't seem to make much difference)
        self._scene_cache_capacity = 50
        self._scene_cache = {}
        self._scene_cache_keys = []

        # Update the timeline buttons to correspond with a completely
        # uninitialised state.
        self._set_timeline_buttons(first=False, previous=False, next=False, last=False)

        self._deferred_fit_in_view.connect(self._fit_in_view,
                                           Qt.QueuedConnection)
        self._deferred_fit_in_view.emit()

        # This is used to store a timer which controls how fast updates happen when the play button is pressed.
        self._play_timer = None

        # updates the view
        self._refresh_view.connect(self._refresh_tf_graph)

        context.add_widget(self._widget)

        self._force_refresh = False

    def tree_cb(self, tree):
        """Called whenever a message is received on the topic being subscribed to.

        :param :class:`BehaviourTree` tree: representation of a behaviour tree
        """
        # If not viewing a bag, append messages to the list. Otherwise, they are
        # ignored. TODO: store messages somehow?
        if not self._viewing_bag:
            # If this is the first message received, update buttons to allow navigation
            if len(self.message_list) == 1:
                self._set_timeline_buttons(first=True, previous=True)
            self.message_list.append(tree)

            # If the user is browsing the timeline, update the number of new
            # messages since they stopped looking at the latest tree.
            if self._browsing_timeline:
                self._new_messages += 1
                self._update_label_new_messages()

        # Only refresh the view if the user is looking at the latest tree
        if not self._viewing_bag and not self._browsing_timeline:
            self.current_message = len(self.message_list) - 1
            self._refresh_view.emit()

    def get_current_message(self):
        """Get the message in the list or bag that is being viewed that should be
        displayed.

        """
        if self._timeline_listener:
            try:

                return self._timeline_listener.msg
            except KeyError:
                pass

        return py_trees_msgs.BehaviourTree()

    def _choose_topic(self, index):
        """Updates the topic that is subscribed to based on changes to the combo box
        text. If the topic is unchanged, nothing will happnen. Otherwise, the
        old subscriber will be unregistered, and a new one initialised for the
        updated topic. If the selected topic corresponds to the unselected
        topic, the subscriber will be unregistered and a new one will not be
        created.

        """
        selected_topic = self._widget.topic_combo_box.currentText()
        if selected_topic != self._empty_topic and self.current_topic != selected_topic:
            self.current_topic = selected_topic
            # destroy the old timeline and clear the scene
            if self._timeline:
                self._timeline.handle_close()
                self._widget.timeline_graphics_view.setScene(None)

            if selected_topic != self._unselected_topic:
                # set up a timeline to track the messages coming from the subscriber
                self._set_dynamic_timeline()

    def _update_combo_topics(self):
        """Update the topics displayed in the combo box that the user can use to select
        which topic they want to listen on for trees, filtered so that only
        topics with the correct message type are shown.

        """
        self._widget.topic_combo_box.clear()
        topic_list = rospy.get_published_topics()

        valid_topics = []
        for topic_path, topic_type in topic_list:
            if topic_type == RosBehaviourTree._expected_type:
                valid_topics.append(topic_path)

        if not valid_topics:
            self._widget.topic_combo_box.addItem(RosBehaviourTree._empty_topic)
            return

        # always add an item which does nothing so that it is possible to listen to nothing.
        self._widget.topic_combo_box.addItem(RosBehaviourTree._unselected_topic)
        for topic in valid_topics:
            self._widget.topic_combo_box.addItem(topic)

    def _update_label_new_messages(self):
        """Update the label with information about the number of new messages. Blank if
        zero new.

        """
        if self._new_messages == 0:
            self._widget.message_label.setText("")
        else:
            formatstr = "{0} new message" + ("" if self._new_messages == 1 else "s")
            self._widget.message_label.setText(formatstr.format(self._new_messages))

    def _set_timeline_buttons(self, first=None, previous=None, next=None, last=None):
        """Allows timeline buttons to be enabled and disabled.
        """
        if first is not None:
            self._widget.first_tool_button.setEnabled(first)
        if previous is not None:
            self._widget.previous_tool_button.setEnabled(previous)
        if next is not None:
            self._widget.next_tool_button.setEnabled(next)
        if last is not None:
            self._widget.last_tool_button.setEnabled(last)

    def _play(self):
        """Start a timer which will automatically call the next function every time its
        duration is up. Only works if the current message is not the final one.

        """
        if not self._play_timer:
            self._play_timer = rospy.Timer(rospy.Duration(1), self._timer_next)

    def _timer_next(self, timer):
        """Helper functions for the timer so that it can call the next function without
        breaking.

        """
        self._next()

    def _stop(self):
        """Stop the play timer, if it exists.
        """
        if self._play_timer:
            self._play_timer.shutdown()
            self._play_timer = None

    def _first(self):
        """Navigate to the first message. Activates the next and last buttons, disables
        first and previous, and refreshes the view. Also changes the state to be
        browsing the timeline.

        """
        self._timeline.navigate_start()

        #self.current_message = 0
        self._set_timeline_buttons(first=False, previous=False, next=True, last=True)
        self._refresh_view.emit()
        self._browsing_timeline = True

    def _previous(self):
        """Navigate to the previous message. Activates the next and last buttons, and
        refreshes the view. If the current message is the second message, then
        the first and previous buttons are disabled. Changes the state to be
        browsing the timeline.

        """
        # if already at the beginning, do nothing
        if self._timeline._timeline_frame.playhead == self._timeline._get_start_stamp():
            return

        # otherwise, go to the previous message
        self._timeline.navigate_previous()
        # now browsing the timeline
        self._browsing_timeline = True
        self._set_timeline_buttons(last=True, next=True)
        # if now at the beginning, disable timeline buttons.
        if self._timeline._timeline_frame.playhead == self._timeline._get_end_stamp():
            self._set_timeline_buttons(next=False, last=False)

        self._refresh_view.emit()

    def _last(self):
        """Navigate to the last message. Activates the first and previous buttons,
        disables next and last, and refreshes the view. The user is no longer
        browsing the timeline after this is called.

        """
        self._timeline.navigate_end()

        # self.current_message = len(self.message_list) - 1
        self._set_timeline_buttons(first=True, previous=True, next=False, last=False)
        self._refresh_view.emit()
        self._browsing_timeline = False
        self._new_messages = 0
        self._update_label_new_messages()

    def _next(self):
        """Navigate to the next message. Activates the first and previous buttons. If
        the current message is the second from last, disables the next and last
        buttons, and stops browsing the timeline.

        """
        # if already at the end, do nothing
        if self._timeline._timeline_frame.playhead == self._timeline._get_end_stamp():
            return

        # otherwise, go to the next message
        self._timeline.navigate_next()
        self._set_timeline_buttons(first=True, previous=True)
        # if now at the end, disable timeline buttons and shutdown the play timer if active
        if self._timeline._timeline_frame.playhead == self._timeline._get_end_stamp():
            self._set_timeline_buttons(next=False, last=False)
            self._browsing_timeline = False
            if self._play_timer:
                self._play_timer.shutdown()

        self._refresh_view.emit()

    def save_settings(self, plugin_settings, instance_settings):
        instance_settings.set_value('auto_fit_graph_check_box_state',
                                    self._widget.auto_fit_graph_check_box.isChecked())
        instance_settings.set_value('highlight_connections_check_box_state',
                                    self._widget.highlight_connections_check_box.isChecked())

    def restore_settings(self, plugin_settings, instance_settings):
        self._widget.auto_fit_graph_check_box.setChecked(
            instance_settings.value('auto_fit_graph_check_box_state', True) in [True, 'true'])
        self._widget.highlight_connections_check_box.setChecked(
            instance_settings.value('highlight_connections_check_box_state', True) in [True, 'true'])
        self.initialized = True
        self._refresh_tf_graph()

    def _refresh_tf_graph(self):
        """Refresh the graph view by regenerating the dotcode from the current message.

        """
        if not self.initialized:
            return
        self._update_graph_view(self._generate_dotcode())

    def _get_dotcode(self, message):
        """Get the dotcode for the given message, checking the cache for dotcode that
        was previously generated, and adding to the cache if it wasn't there.
        Cache replaces LRU.

        Mostly stolen from rqt_bag.MessageLoaderThread

        """
        key = str(message.header.stamp) # stamps are unique
        if key in self._dotcode_cache:
            return self._dotcode_cache[key]

        force_refresh = self._force_refresh
        self._force_refresh = False

        # cache miss
        dotcode = self.dotcode_generator.generate_dotcode(dotcode_factory=self.dotcode_factory,
                                                          tree=message,
                                                          force_refresh=force_refresh)

        self._dotcode_cache[key] = dotcode
        self._dotcode_cache_keys.append(key)

        if len(self._dotcode_cache) > self._dotcode_cache_capacity:
            oldest = self._dotcode_cache_keys[0]
            del self._dotcode_cache[oldest]
            self._dotcode_cache_keys.remove(oldest)

        return dotcode

    def _generate_dotcode(self):
        """Generate dotcode from the current message
        """
        message = self.get_current_message()

        return self._get_dotcode(self.get_current_message())

    def _update_graph_view(self, dotcode):
        if dotcode == self._current_dotcode:
            return
        self._current_dotcode = dotcode
        self._redraw_graph_view()

    def _redraw_graph_view(self):
        key = str(self.get_current_message().header.stamp)
        if key in self._scene_cache:
            self._scene = self._scene_cache[key]
        else: # cache miss
            new_scene = QGraphicsScene()
            new_scene.setBackgroundBrush(Qt.white)

            if self._widget.highlight_connections_check_box.isChecked():
                highlight_level = 3
            else:
                highlight_level = 1

            (nodes, edges) = self.dot_to_qt.dotcode_to_qt_items(self._current_dotcode,
                                                                highlight_level)

            for node_item in nodes.itervalues():
                new_scene.addItem(node_item)
            for edge_items in edges.itervalues():
                for edge_item in edge_items:
                    edge_item.add_to_scene(new_scene)

            new_scene.setSceneRect(new_scene.itemsBoundingRect())

            # put the scene in the cache
            self._scene_cache[key] = new_scene
            self._scene_cache_keys.append(key)

            if len(self._scene_cache) > self._scene_cache_capacity:
                oldest = self._scene_cache_keys[0]
                del self._scene_cache[oldest]
                self._scene_cache_keys.remove(oldest)

            # after construction, set the scene and fit to the view
            self._scene = new_scene

        self._widget.graphics_view.setScene(self._scene)

        if self._widget.auto_fit_graph_check_box.isChecked():
            self._fit_in_view()

    def _resize_event(self, event):
        """Activated when the window is resized. Will re-fit the behaviour tree in the
        window, and update the size of the timeline scene rectangle so that it
        is correctly drawn.

        """
        self._fit_in_view()
        if self._timeline:
            self._timeline.setSceneRect(0, 0, self._widget.timeline_graphics_view.width() - 2, max(self._widget.timeline_graphics_view.height() - 2, self._timeline._timeline_frame._history_bottom))

    def timeline_changed(self):
        """Should be called whenever the timeline changes. At the moment this is only
        used to ensure that the first and previous buttons are correctly
        disabled when a new message coming in on the timeline pushes the
        playhead to be at the first message

        """
        if self._timeline._timeline_frame.playhead == self._timeline._get_start_stamp():
            self._set_timeline_buttons(first=False, previous=False)
        else:
            self._set_timeline_buttons(first=True, previous=True)

    def message_changed(self):
        """This function should be called when the message being viewed changes. Will
        change the current message and update the view. Also ensures that the
        timeline buttons are correctly set for the current position of the
        playhead on the timeline.

        """
        if self._timeline._timeline_frame.playhead == self._timeline._get_end_stamp():
            self._set_timeline_buttons(last=False, next=False)
        else:
            self._set_timeline_buttons(last=True, next=True)

        if self._timeline._timeline_frame.playhead == self._timeline._get_start_stamp():
            self._set_timeline_buttons(first=False, previous=False)
        else:
            self._set_timeline_buttons(first=True, previous=True)

        self._refresh_view.emit()

    def message_cleared(self):
        """This function should be called when the message being viewed was cleared.
        Currently no situation where this happens?

        """
        pass

    def no_right_click_press_event(self, func):
        """Decorator for ignoring right click events on mouse press
        """
        @functools.wraps(func)
        def wrapper(event):
            if event.type() == QEvent.MouseButtonPress and event.button() == Qt.RightButton:
                event.ignore()
            else:
                func(event)
        return wrapper

    def _set_dynamic_timeline(self):
        """Set the timeline to a dynamic timeline, listening to messages on the topic
        selected in the combo box.

        """
        self._timeline = DynamicTimeline(self, publish_clock=False)
        # connect timeline events so that the timeline will update when events happen
        self._widget.timeline_graphics_view.mousePressEvent = self.no_right_click_press_event(self._timeline.on_mouse_down)
        self._widget.timeline_graphics_view.mouseReleaseEvent = self._timeline.on_mouse_up
        self._widget.timeline_graphics_view.mouseMoveEvent = self._timeline.on_mouse_move
        self._widget.timeline_graphics_view.wheelEvent = self._timeline.on_mousewheel
        self._widget.timeline_graphics_view.setScene(self._timeline)

        # Don't show scrollbars - the timeline adjusts to the size of the view
        self._widget.timeline_graphics_view.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self._widget.timeline_graphics_view.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        # Send a resize event so that the timeline knows the size of the view it's in
        self._resize_event(None)

        self._timeline.add_topic(self.current_topic, py_trees_msgs.BehaviourTree)

        # Create a listener for the timeline which will call the emit function
        # on the given signals when the message being viewed changes or is
        # cleared. The message being viewed changing generally happens when the
        # user moves the slider around.
        self._timeline_listener = DynamicTimelineListener(self._timeline, self.current_topic, self._message_changed, self._message_cleared)
        # Need to add a listener to make sure that we can get information about
        # messages that are on the topic that we're interested in.
        self._timeline.add_listener(self.current_topic, self._timeline_listener)

        self._timeline.navigate_end()
        self._timeline._redraw_timeline(None)
        self._timeline.timeline_updated.connect(self.timeline_changed)

    def _set_bag_timeline(self, bag):
        """Set the timeline of this object to a bag timeline, hooking the graphics view
        into mouse and wheel functions of the timeline.

        """
        self._timeline = BagTimeline(self, publish_clock=False)
        # connect timeline events so that the timeline will update when events happen
        self._widget.timeline_graphics_view.mousePressEvent = self.no_right_click_press_event(self._timeline.on_mouse_down)
        self._widget.timeline_graphics_view.mouseReleaseEvent = self._timeline.on_mouse_up
        self._widget.timeline_graphics_view.mouseMoveEvent = self._timeline.on_mouse_move
        self._widget.timeline_graphics_view.wheelEvent = self._timeline.on_mousewheel
        self._widget.timeline_graphics_view.setScene(self._timeline)

        # Don't show scrollbars - the timeline adjusts to the size of the view
        self._widget.timeline_graphics_view.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self._widget.timeline_graphics_view.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        # Send a resize event so that the timeline knows the size of the view it's in
        self._resize_event(None)

        self._timeline.add_bag(bag)
        # Create a listener for the timeline which will call the emit function
        # on the given signals when the message being viewed changes or is
        # cleared. The message being viewed changing generally happens when the
        # user moves the slider around.
        self._timeline_listener = TimelineListener(self._timeline, self.current_topic, self._message_changed, self._message_cleared)
        # Need to add a listener to make sure that we can get information about
        # messages that are on the topic that we're interested in.
        self._timeline.add_listener(self.current_topic, self._timeline_listener)
        # Go to the first message in the timeline of the bag.
        self._timeline.navigate_start()

    def _load_bag(self, file_name=None):
        """Load a bag from file. If no file name is given, a dialogue will pop up and
        the user will be asked to select a file. If the bag file selected
        doesn't have any valid topic, nothing will happen. If there are valid
        topics, we load the bag and add a timeline for managing it.

        """
        if file_name is None:
            file_name, _ = QFileDialog.getOpenFileName(
                self._widget,
                self.tr('Open trees from bag file'),
                None,
                self.tr('ROS bag (*.bag)'))
        if file_name is None or file_name == "":
            return

        rospy.loginfo("Reading bag from {0}".format(file_name))
        bag = rosbag.Bag(file_name, 'r')
        # ugh...
        topics = bag.get_type_and_topic_info()[1].keys()
        types = []
        for i in range(0,len(bag.get_type_and_topic_info()[1].values())):
            types.append(bag.get_type_and_topic_info()[1].values()[i][0])

        tree_topics = [] # only look at the first matching topic
        for ind, tp in enumerate(types):
            if tp == 'py_trees_msgs/BehaviourTree':
                tree_topics.append(topics[ind])

        if len(tree_topics) == 0:
            rospy.logerr('Requested bag did not contain any valid topics.')
            return

        self.message_list = []
        self._viewing_bag = True
        rospy.loginfo('Reading behaviour trees from topic {0}'.format(tree_topics[0]))
        for topic, msg, t in bag.read_messages(topics=[tree_topics[0]]):
            self.message_list.append(msg)

        self.current_topic = tree_topics[0]
        #self.current_message = len(self.message_list) - 1
        self._set_timeline_buttons(first=True, previous=True, next=False, last=False)
        self._set_bag_timeline(bag)
        self._refresh_view.emit()

    def _load_dot(self, file_name=None):
        if file_name is None:
            file_name, _ = QFileDialog.getOpenFileName(
                self._widget,
                self.tr('Open tree from DOT file'),
                None,
                self.tr('DOT graph (*.dot)'))
            if file_name is None or file_name == '':
                return

        try:
            fhandle = open(file_name, 'rb')
            dotcode = fhandle.read()
            fhandle.close()
        except IOError:
            return

        self._update_graph_view(dotcode)

    def _fit_in_view(self):
        self._widget.graphics_view.fitInView(self._scene.itemsBoundingRect(),
                                             Qt.KeepAspectRatio)

    def _save_dot(self):
        file_name, _ = QFileDialog.getSaveFileName(self._widget,
                                                   self.tr('Save as DOT'),
                                                   'frames.dot',
                                                   self.tr('DOT graph (*.dot)'))
        if file_name is None or file_name == '':
            return

        file = QFile(file_name)
        if not file.open(QIODevice.WriteOnly | QIODevice.Text):
            return

        file.write(self._current_dotcode)
        file.close()

    def _save_svg(self):
        file_name, _ = QFileDialog.getSaveFileName(
            self._widget,
            self.tr('Save as SVG'),
            'frames.svg',
            self.tr('Scalable Vector Graphic (*.svg)'))
        if file_name is None or file_name == '':
            return

        generator = QSvgGenerator()
        generator.setFileName(file_name)
        generator.setSize((self._scene.sceneRect().size() * 2.0).toSize())

        painter = QPainter(generator)
        painter.setRenderHint(QPainter.Antialiasing)
        self._scene.render(painter)
        painter.end()

    def _save_image(self):
        file_name, _ = QFileDialog.getSaveFileName(
            self._widget,
            self.tr('Save as image'),
            'frames.png',
            self.tr('Image (*.bmp *.jpg *.png *.tiff)'))
        if file_name is None or file_name == '':
            return

        img = QImage((self._scene.sceneRect().size() * 2.0).toSize(),
                     QImage.Format_ARGB32_Premultiplied)
        painter = QPainter(img)
        painter.setRenderHint(QPainter.Antialiasing)
        self._scene.render(painter)
        painter.end()
        img.save(file_name)

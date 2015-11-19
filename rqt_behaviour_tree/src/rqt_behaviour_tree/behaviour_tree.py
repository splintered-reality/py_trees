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
import py_trees.msg as py_trees_msgs

from python_qt_binding import loadUi
from python_qt_binding.QtCore import QFile, QIODevice, QObject, Qt, Signal
from python_qt_binding.QtGui import QFileDialog, QGraphicsScene, QIcon, QImage, QPainter, QWidget, QShortcut, QKeySequence
from python_qt_binding.QtSvg import QSvgGenerator
from qt_dotgraph.pydotfactory import PydotFactory
from qt_dotgraph.dot_to_qt import DotToQtGenerator
from rqt_graph.interactive_graphics_view import InteractiveGraphicsView

from .dotcode_behaviour import RosBehaviourTreeDotcodeGenerator


class RosBehaviourTree(QObject):

    _deferred_fit_in_view = Signal()
    _refresh_view = Signal()

    def __init__(self, context):
        super(RosBehaviourTree, self).__init__(context)
        self.setObjectName('RosBehaviourTree')

        self.initialized = False
        self._current_dotcode = None # dotcode for the tree that is currently displayed
        self._viewing_bag = False # true if a bag file is loaded
        # True if next or previous buttons are pressed. Reset if the tree being
        # viewed is the last one in the list.
        self._browsing_timeline = False
        self.message_list = [py_trees_msgs.BehaviourTree()] # list of all the messages received so far.
        self.current_message = 0 # index of the currently viewed message in the list. 

        self._widget = QWidget()

        # factory builds generic dotcode items
        self.dotcode_factory = PydotFactory()
        # self.dotcode_factory = PygraphvizFactory()
        # generator builds rosgraph
        self.dotcode_generator = RosBehaviourTreeDotcodeGenerator()
        self.behaviour_sub = rospy.Subscriber('/gopher_deliveries/log/tree', py_trees_msgs.BehaviourTree, self.tree_cb)

        # dot_to_qt transforms into Qt elements using dot layout
        self.dot_to_qt = DotToQtGenerator()

        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('rqt_behaviour_tree'), 'resource', 'RosBehaviourTree.ui')
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
        self._widget.refresh_graph_push_button.setIcon(QIcon.fromTheme('view-refresh'))
        self._widget.refresh_graph_push_button.pressed.connect(self._clear_graph)

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
        preemacsous_shortcut_emacs = QShortcut(QKeySequence("Ctrl+b"), self._widget)
        preemacsous_shortcut_emacs.activated.connect(self._widget.previous_tool_button.pressed)
        first_shortcut_emacs = QShortcut(QKeySequence("Ctrl+a"), self._widget)
        first_shortcut_emacs.activated.connect(self._widget.first_tool_button.pressed)
        last_shortcut_emacs = QShortcut(QKeySequence("Ctrl+e"), self._widget)
        last_shortcut_emacs.activated.connect(self._widget.last_tool_button.pressed)

        self._set_timeline_buttons(first=False, previous=False, next=False, last=False)

        self._deferred_fit_in_view.connect(self._fit_in_view,
                                           Qt.QueuedConnection)
        self._deferred_fit_in_view.emit()

        self._refresh_view.connect(self._refresh_tf_graph)
        self._play_timer = None

        context.add_widget(self._widget)

        self._force_refresh = False

    def tree_cb(self, tree):
        if not self._viewing_bag:
            self.message_list.append(tree)

        if not self._viewing_bag and not self._browsing_timeline:
            self.current_message = len(self.message_list) - 1
            self._refresh_view.emit()

    def _clear_graph(self):
        self._scene.clear()
        self._current_dotcode = None
        self._refresh_view.emit()

    def _set_timeline_buttons(self, first=None, previous=None, next=None, last=None):
        if first is not None:
            self._widget.first_tool_button.setEnabled(first)
        if previous is not None:
            self._widget.previous_tool_button.setEnabled(previous)
        if next is not None:
            self._widget.next_tool_button.setEnabled(next)
        if last is not None:
            self._widget.last_tool_button.setEnabled(last)

    def _play(self):
        self._play_timer = rospy.Timer(rospy.Duration(1), self._timer_next)

    def _timer_next(self, timer):
        self._next()

    def _stop(self):
        if self._play_timer:
            self._play_timer.shutdown()

    def _first(self):
        self.current_message = 0
        self._set_timeline_buttons(first=False, previous=False, next=True, last=True)
        self._refresh_view.emit()

    def _previous(self):
        rospy.loginfo("previous! - len is {0}, cur is {1}".format(len(self.message_list), self.current_message))

        self.current_message -= 1 # point to the next message in the list
        if not self._widget.next_tool_button.isEnabled():
            self._set_timeline_buttons(next=True, last=True)

        # If already at the end of the list, or just reached it with this bump,
        # disable the button and reset the index to the end
        if self.current_message <= 0:
            self.current_message = 0
            self._set_timeline_buttons(first=False, previous=False)

        self._refresh_view.emit()

    def _last(self):
        self.current_message = len(self.message_list) - 1
        self._set_timeline_buttons(first=True, previous=True, next=False, last=False)
        self._refresh_view.emit()

    def _next(self):
        rospy.loginfo("next! - len is {0}, cur is {1}".format(len(self.message_list), self.current_message))

        self.current_message += 1 # point to the next message in the list
        if not self._widget.previous_tool_button.isEnabled():
            self._set_timeline_buttons(first=True, previous=True)

        # If already at the end of the list, or just reached it with this bump,
        # disable the button and reset the index to the end
        if self.current_message >= len(self.message_list) - 1:
            self.current_message = len(self.message_list) - 1
            self._set_timeline_buttons(next=False, last=False)
            # the play timer will call this function each time it ticks, so once
            # we reach the end of the bag, stop it.
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
        if not self.initialized:
            return
        self._update_graph_view(self._generate_dotcode())

    def _generate_dotcode(self):
        force_refresh = self._force_refresh
        self._force_refresh = False
        return self.dotcode_generator.generate_dotcode(dotcode_factory=self.dotcode_factory,
                                                       tree=self.message_list[self.current_message],
                                                       force_refresh=force_refresh)

    def _update_graph_view(self, dotcode):
        if dotcode == self._current_dotcode:
            return
        self._current_dotcode = dotcode
        self._redraw_graph_view()

    def _generate_tool_tip(self, url):
        return url

    def _redraw_graph_view(self):
        self._scene.clear()

        if self._widget.highlight_connections_check_box.isChecked():
            highlight_level = 3
        else:
            highlight_level = 1

        (nodes, edges) = self.dot_to_qt.dotcode_to_qt_items(self._current_dotcode,
                                                            highlight_level)

        for node_item in nodes.itervalues():
            self._scene.addItem(node_item)
        for edge_items in edges.itervalues():
            for edge_item in edge_items:
                edge_item.add_to_scene(self._scene)

        self._scene.setSceneRect(self._scene.itemsBoundingRect())
        if self._widget.auto_fit_graph_check_box.isChecked():
            self._fit_in_view()

    def _load_bag(self, file_name=None):
        if file_name is None:
            file_name, _ = QFileDialog.getOpenFileName(
                self._widget,
                self.tr('Open trees from bag file'),
                None,
                self.tr('ROS bag (*.bag)'))
        if file_name is None or file_name == "":
            return

        bag = rosbag.Bag(file_name, 'r')
        # ugh...
        topics = bag.get_type_and_topic_info()[1].keys()
        types = []
        for i in range(0,len(bag.get_type_and_topic_info()[1].values())):
            types.append(bag.get_type_and_topic_info()[1].values()[i][0])

        tree_topics = [] # only look at the first matching topic
        for ind, tp in enumerate(types):
            if tp == 'py_trees/BehaviourTree':
                tree_topics.append(topics[ind])

        if len(tree_topics) == 0:
            rospy.logerr('Requested bag did not contain any valid topics.')
            return
                
        self.message_list = []
        self._viewing_bag = True
        rospy.loginfo('Reading behaviour trees from topic {0}'.format(tree_topics[0]))
        for topic, msg, t in bag.read_messages(topics=[tree_topics[0]]):
            self.message_list.append(msg)
            
        self.current_message = len(self.message_list) - 1
        self._refresh_view.emit()
        self._set_timeline_buttons(first=True, previous=True, next=False, last=False)
        
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

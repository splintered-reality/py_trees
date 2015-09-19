#
# License: Yujin
#
##############################################################################
# Description
##############################################################################

"""
.. module:: navigation
   :platform: Unix
   :synopsis: Navigation related behaviours

Oh my spaghettified magnificence,
Bless my noggin with a tickle from your noodly appendages!

----

"""

##############################################################################
# Imports
##############################################################################

import geometry_msgs.msg as geometry_msgs
import py_trees
import rocon_python_comms
import rospy
import std_msgs.msg as std_msgs
import std_srvs.srv as std_srvs
import tf
from gopher_configuration.parameters import Parameters

##############################################################################
# Behaviours
##############################################################################


class ClearCostmaps(py_trees.Behaviour):
    """
    Clear costmaps. Always returns success, even if it didn't successfully
    call the service, since if it fails, it's not usually critical. If we
    ever need that critical behaviour, make it an option in the init args.
    """
    def __init__(self, name):
        super(ClearCostmaps, self).__init__(name)
        self.parameters = Parameters()

    def update(self):
        rospy.loginfo("Gopher Behaviours : clearing costmaps [%s]" % (type(self).__name__))
        try:
            clear_costmaps_service = rospy.ServiceProxy(self.parameters.services.clear_costmaps, std_srvs.Empty)
            clear_costmaps_service()
        except rospy.ServiceException as e:
            rospy.logwarn("Gopher Behaviours : failed to clear costmaps, wrong service name name? [%s][%s][%s]" % (self.parameters.services.clear_costmap, type(self).__name__, str(e)))
        except rospy.ROSInterruptException:
            rospy.logwarn("Gopher Behaviours : interrupted while trying to clear costmaps, probably ros shutting down [%s]" % type(self).__name__)
        return py_trees.Status.SUCCESS


class InitPose(py_trees.Behaviour):
    """
    Init the pose. Usually used in conjunction with switching maps.
    """
    def __init__(self, name, initial_pose, topic_name):
        """
        He is a mere noodly appendage - don't expect him to be smart and check if
        the pose matches the semantics, or if the topics actually connect.

        A pastafarian at a higher level should take care of that before construction.

        :param geometry_msgs.PoseStamped initial_pose: where to init after loading the map.
        :param str topic_name:
        """
        super(InitPose, self).__init__(name)
        self.parameters = Parameters()
        self.initial_pose = initial_pose
        self.publisher = rocon_python_comms.Publisher(topic_name, geometry_msgs.PoseWithCovarianceStamped, queue_size=1)

    def update(self):
        if self.publisher.is_ready():
            rospy.loginfo("Gopher Deliveries : initialising the pose {x: %s, y: %s, theta: %s}" % (self.initial_pose.x, self.initial_pose.y, self.initial_pose.theta))
            self.publisher.publish(self.to_msg_pose_with_covariance_stamped(self.initial_pose))
            return py_trees.Status.SUCCESS
        else:
            return py_trees.Status.RUNNING

    def to_msg_pose_with_covariance_stamped(self, pose_2d):
        msg = geometry_msgs.PoseWithCovarianceStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.parameters.frames.map
        quaternion = tf.transformations.quaternion_from_euler(0, 0, self.initial_pose.theta)
        msg_pose = geometry_msgs.Pose()
        msg_pose.position.x = self.initial_pose.x
        msg_pose.position.y = self.initial_pose.y
        msg_pose.position.z = 0.0
        msg_pose.orientation.x = quaternion[0]
        msg_pose.orientation.y = quaternion[1]
        msg_pose.orientation.z = quaternion[2]
        msg_pose.orientation.w = quaternion[3]
        msg.pose.pose = msg_pose
        return msg


class SwitchMap(py_trees.Behaviour):
    """
    Switch the map. This doesn't check semantics for anything, some component
    should do that before getting to this behaviour and feed him with the
    required information to execute. This keeps the behaviour a bit simpler.

    Remember : This behaviour assumes input values are semantically correct!
               Do your checking before init'ing!
    """
    def __init__(self, name, map_filename, topic_name):
        """
        He is a mere noodly appendage - don't expect him to be smart and check if
        the map exists in semantics, or on the filesystem, or if the topics
        actually connect.

        A pastafarian at a higher level should take care of that before construction.

        :param str map_filename: full pathname to the dslam map.
        :param str topic_name:
        """
        super(SwitchMap, self).__init__(name)
        self.dslam_map = std_msgs.String(map_filename)
        self.parameters = Parameters()
        self.publisher = rocon_python_comms.Publisher(topic_name, std_msgs.String, queue_size=1)

    def update(self):
        if self.publisher.is_ready():
            rospy.loginfo("Gopher Behaviours : switching map [%s]" % self.dslam_map.data)
            self.publisher.publish(self.dslam_map)
            return py_trees.Status.SUCCESS
        else:
            return py_trees.Status.RUNNING

#
# License: Yujin
#
##############################################################################
# Description
##############################################################################

"""
.. module:: utilities
   :platform: Unix
   :synopsis: Tools for debugging and development.

Oh my spaghettified magnificence,
Bless my noggin with a tickle from your noodly appendages!

----

"""

##############################################################################
# Imports
##############################################################################

import geometry_msgs.msg as geometry_msgs
import gopher_configuration
import gopher_semantics
import os
import rospkg
import rospy
import tf

##############################################################################
# Methods
##############################################################################


def get_gopher_home():
    '''
      Retrieve the location of the gopher home directory.
      If it does not exist, create a new directory and return this path.

      .. todo:: move this to a common library (currently also used by gopher_customisation)

      :return: the gopher home directory (path)
      :rtype str:
    '''
    gopher_home = os.path.join(rospkg.get_ros_home(), 'gopher')
    if not os.path.isdir(gopher_home):
        os.makedirs(gopher_home)
    return os.path.join(rospkg.get_ros_home(), 'gopher')


def get_world_name(filename):
    """
    Extract the world name from a .dslam filename.
    """
    world_name, unused_extension = os.path.splitext(os.path.basename(filename))
    return world_name


def is_map_file(filename):
    unused_f, ext = os.path.splitext(filename)
    return ext == '.dslam'


def get_installed_maps():
    """
    Looks for maps in the gopher home and returns them as a list.
    :returns: dictionary of map names and their paths.
    :rtype: { str : path }
    """
    map_home = os.path.join(get_gopher_home(), 'maps')
    map_filenames = []
    for (unused_dirpath, unused_dirnames, filenames) in os.walk(map_home):
        map_filenames.extend([f for f in filenames if is_map_file(f)])
    return {os.path.splitext(os.path.basename(filename))[0]: os.path.join(map_home, filename) for filename in map_filenames}


def get_installed_worlds(maps=None):
    """
    Looks for maps in the gopher home and checks them off against the
    semantically defined list of worlds.

    :returns: dictionary of world names and their corresponding map filenames
    :rtype: { str : filename }
    """
    if maps is None:
        maps = get_installed_maps()
    gopher = gopher_configuration.Configuration()
    semantics = gopher_semantics.Semantics(gopher.namespaces.semantics)
    worlds = {}
    for world in semantics.worlds:
        if world in maps:
            worlds[world] = maps[world]
        else:
            worlds[world] = None
    return worlds


def msg_origin_pose(frame_id):
    msg = geometry_msgs.PoseWithCovarianceStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = frame_id
    msg_pose = geometry_msgs.Pose()
    msg_pose.position.x = 0.0
    msg_pose.position.y = 0.0
    msg_pose.position.z = 0.0
    msg_pose.orientation.x = 0
    msg_pose.orientation.y = 0
    msg_pose.orientation.z = 0
    msg_pose.orientation.w = 1
    msg.pose.pose = msg_pose
    return msg


def msg_pose2d_to_pose_with_covariance_stamped(pose_2d, frame_id):
    msg = geometry_msgs.PoseWithCovarianceStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = frame_id
    quaternion = tf.transformations.quaternion_from_euler(0, 0, pose_2d.theta)
    msg_pose = geometry_msgs.Pose()
    msg_pose.position.x = pose_2d.x
    msg_pose.position.y = pose_2d.y
    msg_pose.position.z = 0.0
    msg_pose.orientation.x = quaternion[0]
    msg_pose.orientation.y = quaternion[1]
    msg_pose.orientation.z = quaternion[2]
    msg_pose.orientation.w = quaternion[3]
    msg.pose.pose = msg_pose
    return msg

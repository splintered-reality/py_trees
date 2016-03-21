#!/usr/bin/env python
#
# License: Yujin
#
##############################################################################
# Description
##############################################################################

"""
.. module:: transform_utilities
   :platform: Unix
   :synopsis: The unparking logic.

Oh my spaghettified magnificence,
Bless my noggin with a tickle from your noodly appendages!

----

"""

##############################################################################
# Imports
##############################################################################

import geometry_msgs.msg as geometry_msgs
import math
import numpy
import tf.transformations

##############################################################################
# Methods
##############################################################################


def unit_vector(vector):
    """
    The unit vector.

    :param [] vector: list of float length three
    """
    return vector / numpy.linalg.norm(vector)


def angle_between(vector_one, vector_two):
    """
    Angle between two vectors.

    :param [] vector_one: list of float length three
    :param [] vector_two: list of float length three
    """
    unit_vector_one = unit_vector(vector_one)
    unit_vector_two = unit_vector(vector_two)
    # http://stackoverflow.com/questions/2827393/angles-between-two-n-dimensional-vectors-in-python
    return numpy.arccos(numpy.clip(numpy.dot(unit_vector_one, unit_vector_two), -1, 1))


def norm_from_geometry_msgs_pose(pose):
    """
    Compute the norm of the translation part of the pose (i.e.the distance).
    """
    distance = math.sqrt(pose.position.x * pose.position.x +
                         pose.position.y * pose.position.y +
                         pose.position.z * pose.position.z)
    return distance


def angle_from_geometry_msgs_quaternion(quaternion):
    """
    Get the angle-axis angle from the given quaternion.
    """
    angle = 0.0
    return angle


def transform_from_geometry_msgs_pose(pose):
    """
    Convert a geometry_msgs/Pose object into a numpy homogenous 4x4 matrix.

    :param geometry_msgs/Pose pose: incoming pose
    :returns: numpy homogenous matrix representing the transform (inverse of the pose matrix)

    .. seealso:: `Transform Cheat Sheet`_

    .. _Transform Cheat Sheet: https://docs.google.com/document/d/1HRgc0avn08vL_LGN8-SuiaaSQUYt4HbBwwZtVOGzMIo/edit
    """
    translation = [pose.position.x, pose.position.y, pose.position.z]
    translation_homogenous_matrix = tf.transformations.translation_matrix(translation)
    quaternion = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    orientation_homogenous_matrix = tf.transformations.quaternion_matrix(quaternion)
    homogeneous_matrix = numpy.dot(translation_homogenous_matrix, orientation_homogenous_matrix)
    # remember transform is the inverse of pose
    return tf.transformations.inverse_matrix(homogeneous_matrix)


def get_relative_pose(pose_b_rel_c, pose_a_rel_c):
    """
    Compute pose_b_rel_a given a pair of geometry_msg/Pose objects.

    :param geometry_msgs/Pose pose_b_rel_c:
    :param geometry_msgs/Pose pose_a_rel_c:
    """
    T_b_rel_c = transform_from_geometry_msgs_pose(pose_b_rel_c)
    T_a_rel_c = transform_from_geometry_msgs_pose(pose_a_rel_c)
    T_b_rel_a = tf.transformations.concatenate_matrices(T_b_rel_c, tf.transformations.inverse_matrix(T_a_rel_c))
    pose_b_rel_a = tf.transformations.inverse_matrix(T_b_rel_a)

    # lists
    quaternion = tf.transformations.quaternion_from_matrix(pose_b_rel_a)
    translation = tf.transformations.translation_from_matrix(pose_b_rel_a)
    # convert to geometry_msgs.Pose
    geometry_msgs_pose_b_rel_a = geometry_msgs.Pose()
    geometry_msgs_pose_b_rel_a.position = geometry_msgs.Point(translation[0], translation[1], translation[2])
    geometry_msgs_pose_b_rel_a.orientation = geometry_msgs.Quaternion(x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3])

    return geometry_msgs_pose_b_rel_a


def concatenate_poses(pose_a_rel_b, pose_b_rel_c):
    """
    Generate pose_a_rel_c given pose_a_rel_b and pose_b_rel_c
    """
    T_a_rel_b = transform_from_geometry_msgs_pose(pose_a_rel_b)
    T_b_rel_c = transform_from_geometry_msgs_pose(pose_b_rel_c)
    T_a_rel_c = tf.transformations.concatenate_matrices(T_a_rel_b, T_b_rel_c)
    pose_a_rel_c = tf.transformations.inverse_matrix(T_a_rel_c)
    # lists
    quaternion = tf.transformations.quaternion_from_matrix(pose_a_rel_c)
    translation = tf.transformations.translation_from_matrix(pose_a_rel_c)
    # convert to geometry_msgs.Pose
    geometry_msgs_pose_pose_a_rel_c = geometry_msgs.Pose()
    geometry_msgs_pose_pose_a_rel_c.position = geometry_msgs.Point(translation[0], translation[1], translation[2])
    geometry_msgs_pose_pose_a_rel_c.orientation = geometry_msgs.Quaternion(x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3])
    return geometry_msgs_pose_pose_a_rel_c

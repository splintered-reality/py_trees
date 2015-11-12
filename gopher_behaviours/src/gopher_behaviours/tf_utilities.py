#/usr/bin/env python

import tf
import numpy
import geometry_msgs.msg as geometry_msgs

# compute tf1 multiplied by the inverse of tf2 - this gives the transformation
# between the two transformations. Assumes transforms in the form ((x,y,z),
# (x,y,z,w)) where the first part of the tuple is the translation and the second
# part the rotation quaternion. The result is the transform T_tf2_rel_tf1 or
# T^{tf2}_{tf1} or T_tf1_to_tf2

# Based on code at
# http://docs.ros.org/jade/api/tf/html/c++/Transform_8h_source.html
def inverse_times(tf1, tf2):
    # matrix to allow vector-matrix multiplication to generate a vector
    # this is the difference between the translations of the two transforms
    v = numpy.matrix(tf2[0]) - numpy.matrix(tf1[0])
    # create quaternion matrices from the vectors
    q1 = numpy.matrix(tf.transformations.quaternion_matrix(tf1[1]))
    q2 = numpy.matrix(tf.transformations.quaternion_matrix(tf2[1]))
    # difference between the quaternions of the two transforms - relative
    # rotation
    diffq = tf.transformations.quaternion_from_matrix(numpy.transpose(q1) * q2)
    # strip row+column 4 from q1 to multiply with vector
    q1nonhom = q1[numpy.ix_([0, 1, 2], [0, 1, 2])]
    difft = v * q1nonhom

    return (tuple(numpy.array(difft)[0]), tuple(diffq))


# convert a transform to a human readable string
def human_transform(t, oneline=False):
    if oneline:
        return "(x: {0}, y: {1}, yaw: {2})".format(str(t[0][0]), str(t[0][1]), str(numpy.degrees(tf.transformations.euler_from_quaternion(t[1])[2])))
    return "x translation: " + str(t[0][0]) + "\ny translation: " + str(t[0][1]) + "\n yaw: " + str(numpy.degrees(tf.transformations.euler_from_quaternion(t[1])[2]))


def transform_to_pose(t):
    pose = geometry_msgs.Pose()
    pose.position.x = t[0][0]
    pose.position.y = t[0][1]
    pose.position.z = t[0][2]
    pose.orientation.x = t[1][0]
    pose.orientation.y = t[1][1]
    pose.orientation.z = t[1][2]
    pose.orientation.w = t[1][3]

    return pose


def transform_to_transform_stamped(t):
    stamped = geometry_msgs.TransformStamped()

    stamped.transform.translation.x = t[0][0]
    stamped.transform.translation.y = t[0][1]
    stamped.transform.translation.z = t[0][2]

    stamped.transform.rotation.x = t[1][0]
    stamped.transform.rotation.y = t[1][1]
    stamped.transform.rotation.z = t[1][2]
    stamped.transform.rotation.w = t[1][3]

    return stamped


# get the bearing of a transform in the coordinate frame of the transform which
# the given transform is relative to, i.e. given T_homebase_to_dock, this
# function will give you the bearing of the dock from the homebase, in the coordinate frame of the homebase
def transform_bearing(transform):
    # compute the angle between the x axis and the translation vector of the other transform
    xaxis = [1, 0]
    # the translation is the position of the other point of interest in the
    # parent frame
    translation = [transform[0][0], transform[0][1]]
    cross = numpy.cross(xaxis, translation)  # the sign tells us whether this is a positive or negative rotation
    # tan formula from https://newtonexcelbach.wordpress.com/2014/03/01/the-angle-between-two-vectors-python-version/
    return numpy.sign(cross) * numpy.arctan2(numpy.linalg.norm(cross), numpy.dot(xaxis, translation))


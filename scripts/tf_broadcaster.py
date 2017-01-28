#!/usr/bin/env python

"""Tf broadcaster for lidar mount."""

import rospy
import tf
import tf2_ros
import math
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float32

AXEL_HEIGHT = 0.02  # Approximate height of the rotational axis.
BEAM_HEIGHT = 0.05  # Approximate height of the laser.


def handle_platform_angle(msg):
    """Transform the angle received from encoders into TF."""
    br = tf2_ros.TransformBroadcaster()

    #  Create and initialize the tf message.
    t = TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "axel"
    t.child_frame_id = "lidar_mount"

    # Convert angle of encoder from 4096th of a rotation to radians.
    tilt = msg.data * 2.0 * math.pi / 4096

    #  Convert the angle from the encoder into a quaternion and assign to t.
    quat = tf.transformations.quaternion_from_euler(0, tilt, 0)
    t.transform.rotation.x = quat[0]
    t.transform.rotation.y = quat[1]
    t.transform.rotation.z = quat[2]
    t.transform.rotation.w = quat[3]

    #  Broadcast the transform.
    br.sendTransform(t)


if __name__ == '__main__':
    rospy.init_node('lidar_tf_broadcaster')

    #  Register encoder angle handler.
    rospy.Subscriber('lidar_mount_angle', Float32, handle_platform_angle)

    #  Spin forever.
    rospy.spin()
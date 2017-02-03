#!/usr/bin/env python

"""Tf broadcaster for lidar mount."""

import rospy
import math
import tf
import tf2_ros
from std_msgs.msg import Float32
from geometry_msgs.msg import TransformStamped


def handle_platform_angle(msg):
    """Transform the angle received from encoders into TF."""
    br = tf2_ros.TransformBroadcaster()

    #  Create and initialize the tf message.
    t = TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "lidar_mount/axel"
    t.child_frame_id = "lidar_mount/platform"

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
    rospy.logdebug("Published lidar platform angle to tf.")


if __name__ == '__main__':
    rospy.init_node('lidar_tf_broadcaster')

    #  Register encoder angle handler.
    topic_name = '/lidar_mount/encoder_angle'
    rospy.Subscriber(topic_name, Float32, handle_platform_angle)

    #  Spin forever.
    rospy.spin()

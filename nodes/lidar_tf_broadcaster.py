#!/usr/bin/env python
import rospy
import tf
import tf2_ros
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float32

AXEL_HEIGHT = 0.02 #  Approximate height of the rotational axis.
BEAM_HEIGHT = 0.10 #  Approximate height of the laser.


def handle_platform_angle(msg):
    #  Initialize the broadcaster.
    br = tf2_ros.TransformBroadcaster()
    
    #  Create and initialize the tf message.
    t = TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "lidar_mount"
    t.child_frame_id = "axel"

    #  Convert the angle from the encoder into a quaternion and assign to t.
    quat = tf.transformations.quaternion_from_euler(0, msg.data, 0)
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

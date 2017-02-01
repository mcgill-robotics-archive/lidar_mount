#!/usr/bin/env python

"""Sample lidar scan service client."""

import rospy
from laser_assembler.srv import AssembleScans
from sensor_msgs.msg import PointCloud

if __name__ == '__main__':
    rospy.init_node("test_client")
    rospy.wait_for_service("assemble_scans")

    while True:
        r = rospy.Rate(1)
        pub = rospy.Publisher('lidar_pc_pub', PointCloud, queue_size=5)
        try:
            scanner_srv = rospy.ServiceProxy('assemble_scans', AssembleScans)
            resp = scanner_srv(rospy.Time(0, 0), rospy.get_rostime())
            pub.publish(resp.cloud)
        except KeyboardInterrupt as er:
            break
        r.sleep()

#!/usr/bin/env python
import rospy
from laser_assembler.srv import *
from sensor_msgs.msg import PointCloud

if __name__ == '__main__':
    rospy.init_node("test_client")
    rospy.wait_for_service("assemble_scans")

    while True:
        r = rospy.Rate(20)
        pub = rospy.Publisher('lidar_pc_pub', PointCloud, queue_size=5)
        try:
            assemble_scans = rospy.ServiceProxy('assemble_scans', AssembleScans)
            resp = assemble_scans(rospy.Time(0,0), rospy.get_rostime())
            pub.publish(resp.cloud)
        except KeyboardInterrupt, er:
            break

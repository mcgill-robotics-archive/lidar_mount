#!/usr/bin/env python

"""Laser scanner module."""

import rospy
from enum import Enum
from std_msgs.msg import Int32
from lidar_mount.srv import LidarScan, LidarScanResponse


class ScanningStates(Enum):
    """States machine states."""

    IDLE = 1
    SCAN_UP = 2
    SCAN_DOWN = 3


class LaserScanner(object):
    """A state machine that handles the scanning of the lidar."""

    def __init__(self):
        """Initialize state machine and set up service."""
        self.state = ScanningStates.IDLE
        rospy.init_node('laser_scanner')
        self.servo_pub = rospy.Publisher('/lidar_mount/set_servo', Int32,
                                         queue_size=10, latch=True)

        # Setup service to allow toggling between scan and idle.
        srv_name = '/lidar_mount/lidar_scan'
        rospy.Service(srv_name, LidarScan, self.set_scan_state)
        self.scan = False

        # TODO: Make these changeable through dynamic reconfigure.
        self.upper_limit = 15
        self.lower_limit = -30
        self.servo_angle = 0

    def set_scan_state(self, req):
        """Callback which allows us to choose between scanning and idle."""
        change = self.scan is not req.scan
        self.scan = req.scan
        return LidarScanResponse(change)

    def level_laser(self):
        """Set the lidar mount such that the laser is level to the robot."""
        self.servo_angle = 0
        self.servo_pub.publish(Int32(0))

        if self.scan:
            self.state = ScanningStates.SCAN_UP

    def pitch_laser_up(self):
        """Pitch the laser up and check for state transition."""
        self.servo_angle += 1
        self.servo_pub.publish(Int32(self.servo_angle))

        if not self.scan:
            self.state = ScanningStates.IDLE
        elif self.servo_angle > self.upper_limit:
            self.state = ScanningStates.SCAN_DOWN

    def pitch_laser_down(self):
        """Pitch the laser down and check for state transition."""
        self.servo_angle -= 1
        self.servo_pub.publish(Int32(self.servo_angle))
        if not self.scan:
            self.state = ScanningStates.IDLE
        elif self.servo_angle < self.lower_limit:
            self.state = ScanningStates.SCAN_UP

    def evaluate_state(self):
        """Evaluate state and act."""
        switcher = {
            ScanningStates.IDLE: self.level_laser,
            ScanningStates.SCAN_UP: self.pitch_laser_up,
            ScanningStates.SCAN_DOWN: self.pitch_laser_down,
        }

        state_behaviour = switcher.get(self.state)
        return state_behaviour()

    def run(self):
        """Continously evaluate state and pause."""
        r = rospy.Rate(1.0)
        while not rospy.is_shutdown():
            self.evaluate_state()
            r.sleep()


if __name__ == '__main__':
    scanner = LaserScanner()
    scanner.run()

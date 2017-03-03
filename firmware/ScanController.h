/*
 * Scan controller for the lidar mount.
 */
#ifndef LIDAR_MOUNT_SCAN_CONTROLLER
#define LIDAR_MOUNT_SCAN_CONTROLLER


#include <ros.h>
#include <lidar_mount/LidarScan.h>


class ScanController
{
public:
  static bool scan_flag;
  static bool scan_dir_up;
  static int idle;
  static int servo_speed;
  static float pitch_up_lim;
  static float pitch_down_lim;

  /**
   * Callback to be used for the scan mode service.
   */
  static void setScanModeCallback(const lidar_mount::LidarScan::Request & req, lidar_mount::LidarScan::Response & resp);

  /**
   * Returns the appropriate motor speed based on current state of controller.
   */
  static int getMotorCommand(float angle);
};

#endif // LIDAR_MOUNT_SCAN_CONTROLLER

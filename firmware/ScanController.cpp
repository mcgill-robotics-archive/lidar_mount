#include "ScanController.h"

bool ScanController::scan_flag;
bool ScanController::scan_dir_up;
int ScanController::idle;
int ScanController::servo_speed;
float ScanController::pitch_up_lim;
float ScanController::pitch_down_lim;

void ScanController::setScanModeCallback(const lidar_mount::LidarScan::Request & req, lidar_mount::LidarScan::Response & resp)
{
  scan_flag = req.scan;
  resp.scanning = scan_flag;
}

int ScanController::getMotorCommand(float angle)
{
  // Check if we are in scan mode.
  if(scan_flag)
  {
    // Check for limiting conditions.
    if(scan_dir_up && angle < pitch_up_lim && angle > 180)
    {
      scan_dir_up = false;
      servo_speed = -servo_speed;
    }
    else if (!scan_dir_up && angle > pitch_down_lim && angle < 180)
    {
      scan_dir_up = true;
      servo_speed = -servo_speed;
    }

    return idle + servo_speed;
  }
  else
  {
    return idle;
  }
}

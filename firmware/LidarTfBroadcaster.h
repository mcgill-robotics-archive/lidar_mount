#ifndef LIDAR_TF_BR_FIRMWARE
#define LIDAR_TF_BR_FIRMWARE

#include <ros.h>
#include <ros/time.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>

class LidarTfBroadcaster
{
public:
  LidarTfBroadcaster(ros::NodeHandle node_handle);
  
  /*
   * Broadcast tf transform of the lidar from the value
   * recovered by the encoder
   */
  void broadcast_transform_from_tilt(float tilt);
private:
	ros::NodeHandle nh; // Required for nh.now()
	tf::TransformBroadcaster br;
};

#endif // LIDAR_TF_BR_FIRMWARE

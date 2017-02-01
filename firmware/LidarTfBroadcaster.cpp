#include "LidarTfBroadcaster.h"

LidarTfBroadcaster::LidarTfBroadcaster(ros::NodeHandle node_handle)
{
  nh = node_handle;
  br.init(nh);
}

void LidarTfBroadcaster::broadcast_transform_from_tilt(float tilt)
{
  geometry_msgs::TransformStamped t;

  t.header.stamp = nh.now();
  t.header.frame_id = "axel";
  t.child_frame_id = "lidar_mount";

  geometry_msgs::Quaternion q;
  q = tf::createQuaternionFromYaw(tilt);

  t.transform.rotation.x = q.x;
  t.transform.rotation.y = q.z; // Swapped because our
  t.transform.rotation.z = q.y; // angle is pitch, not yaw
  t.transform.rotation.w = q.w;

  br.sendTransform(t);
}

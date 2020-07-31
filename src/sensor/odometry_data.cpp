#include "odometry_data.hpp"

namespace pfuclt::sensor::odometry {

OdometryMeasurement fromRosCustomMsg(const clt_msgs::CustomOdometryConstPtr& msg)
{
  return OdometryMeasurement{msg->header.stamp, msg->pitch, msg->delta_yaw, msg->delta_translation};
}

} // namespace pfuclt::sensor::odometry
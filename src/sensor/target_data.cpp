#include "target_data.hpp"

namespace pfuclt::sensor::target_data{

TargetMeasurement fromRosMsg(const clt_msgs::MeasurementConstPtr &msg){
  ROS_ASSERT_MSG(msg->type == clt_msgs::Measurement::TARGET_CENTER, "Only TARGET_CENTER is supported");
  return TargetMeasurement{msg->id, msg->center.x, msg->center.y, msg->center.z, msg->noise};
}

} // namespace pfuclt::sensor::target_data
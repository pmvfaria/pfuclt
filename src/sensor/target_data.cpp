#include "target_data.hpp"

namespace pfuclt::sensor::target_data{

TargetMeasurement fromRosMsg(const clt_msgs::MeasurementStampedConstPtr &msg){
  ROS_ASSERT_MSG(msg->measurement.type == clt_msgs::Measurement::TARGET_RANGE_BEARING, "Only TARGET_RANGE_BEARING is supported");
  return TargetMeasurement{msg->header.stamp, msg->measurement.id,
                            msg->measurement.range, msg->measurement.bearing, msg->measurement.noise};
}

} // namespace pfuclt::sensor::target_data
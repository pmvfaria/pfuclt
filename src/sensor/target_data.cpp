#include "target_data.hpp"

namespace pfuclt::sensor::target_data{

TargetLikelihood uncertaintyModel(const TargetMeasurement& m) {
  
}

std::unique_ptr<TargetMeasurements> fromRosMsg(const clt_msgs::MeasurementArrayConstPtr &msg){

  auto ts = std::make_unique<TargetMeasurements> (msg->header.frame_id, msg->header.stamp, msg->measurements.size());

  for(const auto& m: msg->measurements){
    ROS_ASSERT_MSG(m.type == clt_msgs::Measurement::TARGET_RANGE_BEARING, "Only TARGET_RANGE_BEARING is supported");
    bool seen = (m.status == clt_msgs::Measurement::PARTIAL || m.status == clt_msgs::Measurement::SEEN);
    ts->measurements.emplace_back(TargetMeasurement{m.id, seen, m.range, m.bearing});
  }

  return ts;
}

} // namespace pfuclt::sensor::target_data
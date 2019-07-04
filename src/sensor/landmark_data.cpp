//
// Created by glawless on 7/30/18.
//

#include "landmark_data.hpp"

namespace pfuclt::sensor::landmark{

std::unique_ptr<LandmarkMeasurements> fromRosMsg(const clt_msgs::MeasurementArrayConstPtr &msg){

  auto lms = std::make_unique<LandmarkMeasurements> (msg->header.frame_id, msg->header.stamp, msg->measurements.size());

  for(const auto& m: msg->measurements){
    ROS_ASSERT_MSG(m.type == clt_msgs::Measurement::LANDMARK_RANGE_BEARING, "Only LANDMARK_RANGE_BEARING is supported");
    bool seen = (m.status == clt_msgs::Measurement::PARTIAL || m.status == clt_msgs::Measurement::SEEN);
    lms->measurements.emplace_back(LandmarkMeasurement{m.id, seen, m.range, m.bearing});
  }

  return lms;
}

} // namespace pfuclt::sensor::landmark
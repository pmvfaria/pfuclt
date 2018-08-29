//
// Created by glawless on 7/30/18.
//

#include "landmark_data.hpp"

namespace pfuclt::sensor::landmark{

LandmarkMeasurements fromRosMsg(const clt_msgs::MeasurementArrayConstPtr &msg){

  auto lms = LandmarkMeasurements{msg->header.frame_id, msg->header.stamp};

  for(const auto& m: msg->measurements){
    ROS_ASSERT_MSG(m.type == clt_msgs::Measurement::LANDMARK_RANGE_BEARING, "Only LANDMARK_RANGE_BEARING is supported");
    lms.measurements.emplace_back(LandmarkMeasurement{m.id, m.range, m.bearing, m.noise});
  }

  return lms;
}

} // namespace pfuclt::sensor::landmark
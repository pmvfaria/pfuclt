#include "measurements_data.hpp"

#include <stdexcept>


namespace pfuclt::sensor::measurement {

Likelihood uncertaintyModel(const Measurement& m)
{
  std::array<double, 3>* K;
  if (m.type == clt_msgs::Measurement::LANDMARK_RANGE_BEARING)
    K = &K_landmarks;
  else if (m.type == clt_msgs::Measurement::TARGET_RANGE_BEARING)
    K = &K_targets;
  else
    throw std::invalid_argument("Only LANDMARK_RANGE_BEARING or TARGET_RANGE_BEARING are supported");

  Likelihood cov;
  cov.dd = (*K)[0] * m.distance;
  cov.aa = (*K)[1] / m.distance;
  cov.ee = (*K)[2] / m.distance;

  return cov;
}

std::unique_ptr<Measurements> fromRosMsg(const clt_msgs::MeasurementArrayConstPtr& msg)
{
  auto ms = std::make_unique<Measurements> (msg->header.frame_id,
                                            msg->header.stamp,
                                            msg->measurements.size());

  for (auto& m : msg->measurements) {
    ROS_ASSERT_MSG(m.type == clt_msgs::Measurement::LANDMARK_RANGE_BEARING ||
                   m.type == clt_msgs::Measurement::TARGET_RANGE_BEARING,
                   "Only LANDMARK_RANGE_BEARING or TARGET_RANGE_BEARING are supported");

    bool seen = (m.status == clt_msgs::Measurement::PARTIAL || m.status == clt_msgs::Measurement::SEEN);
    ms->measurements.emplace_back(Measurement{m.type, m.id - 1, seen, m.distance, m.azimuth, m.elevation});
  }

  return ms;
}

} // namespace pfuclt::sensor::measurement
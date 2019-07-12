//TODO copyright

#ifndef PFUCLT_TARGET_DATA_HPP
#define PFUCLT_TARGET_DATA_HPP

#include <vector>
#include <ros/ros.h>
#include <clt_msgs/MeasurementArray.h>

namespace pfuclt::sensor::target_data{

inline double K_range;
inline double K_bearing;

struct TargetLikelihood{
  double dd, pp;
};

struct TargetMeasurement{
  uint16_t id;
  bool seen;
  double range, bearing;
};

struct TargetMeasurements{
  std::string frame;
  ros::Time stamp;

  std::vector<TargetMeasurement> measurements;

  TargetMeasurements(const std::string &frame_a, const ros::Time& stamp_a, const std::size_t number_measurements)
  : frame(frame_a), stamp(stamp_a) {
    measurements.reserve(number_measurements);
  }
};

TargetLikelihood uncertaintyModel(const TargetMeasurement&);

/**
 * @brief Retrieves target measurement from ROS message
 * @param msg Message which contains the target measurement
 * @return Struct TargetMeasurement containing the measurement
 */
std::unique_ptr<TargetMeasurements> fromRosMsg(const clt_msgs::MeasurementArrayConstPtr &msg);

} // namespace pfuclt::sensor::target_data

#endif //PFUCLT_TARGET_DATA_HPP
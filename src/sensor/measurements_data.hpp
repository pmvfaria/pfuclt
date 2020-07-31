#ifndef PFUCLT_SENSOR_MEASUREMENTS_DATA_HPP
#define PFUCLT_SENSOR_MEASUREMENTS_DATA_HPP

#include <string>
#include <array>
#include <vector>
#include <memory>

#include "ros/ros.h"

#include "clt_msgs/MeasurementArray.h"


namespace pfuclt::sensor::measurement {

// Covariance coefficients
inline std::array<double, 3> K_landmarks;
inline std::array<double, 3> K_targets;

struct Likelihood
{
  double dd, aa, ee;
};

struct Measurement
{
  int type;
  int id;
  bool seen;
  double distance, azimuth, elevation;
};

struct Measurements
{
  std::string frame;
  ros::Time stamp;

  std::vector<Measurement> measurements;

  Measurements(const std::string& frame_a, const ros::Time& stamp_a, const std::size_t num_measurements)
  : frame(frame_a), stamp(stamp_a)
  {
    measurements.reserve(num_measurements);
  }
};

Likelihood uncertaintyModel(const Measurement&);

/**
 * @brief Retrieves measurements from ROS message
 * @param msg Message which contains the measurements
 * @details Ids retrieved from messages should start at 1
 * @return Struct Measurements containing the measurements
 */
std::unique_ptr<Measurements> fromRosMsg(const clt_msgs::MeasurementArrayConstPtr& msg);

} // namespace pfuclt::sensor::measurement

#endif // PFUCLT_SENSOR_MEASUREMENTS_DATA_HPP
//TODO copyright

#ifndef PFUCLT_LANDMARK_DATA_HPP
#define PFUCLT_LANDMARK_DATA_HPP

#include <memory>
#include <vector>
#include <ros/ros.h>
#include <clt_msgs/MeasurementArray.h>

namespace pfuclt::sensor::landmark{

inline double K_range;
inline double K_bearing;

struct LandmarkLikelihood{
  double dd, pp;
};

struct LandmarkMeasurement{
  uint16_t id;
  bool seen;
  double range, bearing;
};

struct LandmarkMeasurements{
  std::string frame;
  ros::Time stamp;

  std::vector<LandmarkMeasurement> measurements;

  LandmarkMeasurements(const std::string &frame_a, const ros::Time& stamp_a, const std::size_t number_measurements)
  : frame(frame_a), stamp(stamp_a) {
    measurements.reserve(number_measurements);
  }
};

LandmarkLikelihood uncertaintyModel(const LandmarkMeasurement&);

/**
 * @brief Retrieves landmark measurements from ROS message
 * @param msg Message which contains the landmark measurements
 * @return Struct LandmarkMeasurements containing the measurements
 */
std::unique_ptr<LandmarkMeasurements> fromRosMsg(const clt_msgs::MeasurementArrayConstPtr &msg);

} // namespace pfuclt::sensor::landmark

#endif //PFUCLT_LANDMARK_DATA_HPP
//TODO copyright

#ifndef PFUCLT_TARGET_DATA_HPP
#define PFUCLT_TARGET_DATA_HPP

#include <vector>
#include <ros/ros.h>
#include <clt_msgs/MeasurementStamped.h>

namespace pfuclt::sensor::target_data{

struct TargetMeasurement{
  ros::Time stamp;
  uint16_t id;
  double range, bearing, noise;
};

/**
 * @brief Retrieves target measurement from ROS message
 * @param msg Message which contains the target measurement
 * @return Struct TargetMeasurement containing the measurement
 */
TargetMeasurement fromRosMsg(const clt_msgs::MeasurementStampedConstPtr &msg);

} // namespace pfuclt::sensor::target_data

#endif //PFUCLT_TARGET_DATA_HPP
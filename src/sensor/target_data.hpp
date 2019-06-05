//TODO copyright

#ifndef PFUCLT_TARGET_DATA_HPP
#define PFUCLT_TARGET_DATA_HPP

#include <vector>
#include <ros/ros.h>
#include <clt_msgs/MeasurementArray.h>

namespace pfuclt::sensor::target_data{

struct TargetMeasurement{
  uint16_t id;
  double x, y, z, noise;
};

TargetMeasurement fromRosMsg(const clt_msgs::MeasurementConstPtr &msg);

} // namespace pfuclt::sensor::target_data

#endif //PFUCLT_TARGET_DATA_HPP
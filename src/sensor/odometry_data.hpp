//TODO copyright

#ifndef PFUCLT_LANDMARK_DATA_HPP
#define PFUCLT_LANDMARK_DATA_HPP

#include <vector>
#include <ros/ros.h>
#include <clt_msgs/CustomOdometry.h>

namespace pfuclt::sensor::odometry{

struct OdometryMeasurement{
  double initial_rotation, translation, final_translation;
};

OdometryMeasurement fromRosCustomMsg(const clt_msgs::CustomOdometryConstPtr &msg);

} // namespace pfuclt::sensor::odometry

#endif //PFUCLT_LANDMARK_DATA_HPP
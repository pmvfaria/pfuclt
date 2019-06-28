//TODO copyright

#ifndef PFUCLT_ODOMETRY_DATA_HPP
#define PFUCLT_ODOMETRY_DATA_HPP

#include <vector>
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <clt_msgs/CustomOdometry.h>

namespace pfuclt::sensor::odometry{

struct OdometryMeasurement{
  const ros::Time stamp;
  const double initial_rotation, translation, final_rotation;
};

/**
 * @brief Retrieves odometry measurement from ROS message
 * @param msg Message which contains the odometry measurements
 * @return Struct OdometryMeasurement containing the measurement
 */
OdometryMeasurement fromRosCustomMsg(const clt_msgs::CustomOdometryConstPtr &msg);

} // namespace pfuclt::sensor::odometry

#endif //PFUCLT_ODOMETRY_DATA_HPP
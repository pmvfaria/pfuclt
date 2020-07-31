#ifndef PFUCLT_SENSOR_ODOMETRY_DATA_HPP
#define PFUCLT_SENSOR_ODOMETRY_DATA_HPP

#include "ros/ros.h"

#include "clt_msgs/CustomOdometry.h"


namespace pfuclt::sensor::odometry {

struct OdometryMeasurement
{
  ros::Time stamp;

  double pitch, delta_yaw, delta_translation;
};

/**
 * @brief Retrieves odometry measurement from ROS message
 * @param msg Message which contains the odometry measurements
 * @return Struct OdometryMeasurement containing the measurement
 */
OdometryMeasurement fromRosCustomMsg(const clt_msgs::CustomOdometryConstPtr& msg);

} // namespace pfuclt::sensor::odometry

#endif // PFUCLT_SENSOR_ODOMETRY_DATA_HPP
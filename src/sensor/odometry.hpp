//TODO copyright

#ifndef PFUCLT_ODOMETRY_HPP
#define PFUCLT_ODOMETRY_HPP

#include <vector>
#include <ros/ros.h>
#include <clt_msgs/CustomOdometry.h>

namespace pfuclt::sensor::odometry{

struct OdometryMeasurement{
  double initial_rotation, translation, final_translation;
  uint8_t robot_id;
};

OdometryMeasurement fromCustomMsg(const clt_msgs::CustomOdometryConstPtr& msg);


class OdometryHandler{

 private:
  std::vector<OdometryMeasurement> queue_;

 public:
  OdometryHandler() = delete;
  OdometryHandler(unsigned long queue_size);
  void callback(const clt_msgs::CustomOdometryConstPtr& msg);


};

} // namespace pfuclt::sensor::odometry

#endif //PFUCLT_ODOMETRY_HPP
//TODO copyright

#ifndef PFUCLT_ODOMETRY_HPP
#define PFUCLT_ODOMETRY_HPP

#include <vector>
#include <ros/ros.h>
#include <clt_msgs/CustomOdometry.h>

namespace pfuclt::sensor::odometry{

struct OdometryMeasurement{
  double initial_rotation, translation, final_translation;
};

OdometryMeasurement fromCustomMsg(const clt_msgs::CustomOdometryConstPtr& msg);


class OdometryHandler{

 private:
  std::vector<OdometryMeasurement> queue_;
  const uint8_t robot_id_;

 public:
  OdometryHandler() = delete;
  OdometryHandler(const std::size_t& queue_size, const uint8_t& robot_id);
  void callback(const clt_msgs::CustomOdometryConstPtr& msg);

};

} // namespace pfuclt::sensor::odometry

#endif //PFUCLT_ODOMETRY_HPP
//TODO copyright

#ifndef PFUCLT_ODOMETRY_H
#define PFUCLT_ODOMETRY_H

#include <vector>
#include <ros/ros.h>
#include <clt_msgs/CustomOdometry.h>

namespace pfuclt::data::odometry{

struct OdometryData{
  double initial_rotation, translation, final_translation;
};

OdometryData fromCustomMsg(const clt_msgs::CustomOdometryConstPtr& msg);


class OdometryHandler{

 private:
  std::vector<OdometryData> queue_;

 public:
  OdometryHandler() = delete;
  OdometryHandler(unsigned long queue_size);
  void callback(const clt_msgs::CustomOdometryConstPtr& msg);


};

} // namespace pfuclt::data

#endif //PFUCLT_ODOMETRY_H
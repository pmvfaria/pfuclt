//
// Created by glawless on 7/30/18.
//

#include "odometry.hpp"

namespace pfuclt::sensor::odometry{

OdometryMeasurement fromCustomMsg(const clt_msgs::CustomOdometryConstPtr &msg, const uint8_t& robot_id){
  return OdometryMeasurement{msg->rot1, msg->rot2, msg->translation, robot_id};
}

OdometryHandler::OdometryHandler(unsigned long queue_size) {
  ROS_ASSERT(queue_size > 0);
  queue_.reserve(queue_size);
}

void OdometryHandler::callback(const clt_msgs::CustomOdometryConstPtr &msg) {
  queue_.emplace_back(fromCustomMsg(msg));
}

} // namespace pfuclt::sensor::odometry
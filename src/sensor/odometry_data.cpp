//
// Created by glawless on 7/30/18.
//

#include "odometry_data.hpp"

namespace pfuclt::sensor::odometry{

OdometryMeasurement fromRosCustomMsg(const clt_msgs::CustomOdometryConstPtr &msg){
  return OdometryMeasurement{msg->rot1, msg->rot2, msg->translation};
}

OdometryHandler::OdometryHandler(const std::size_t& queue_size, const uint8_t& robot_id) : robot_id_(robot_id) {
  ROS_ASSERT(queue_size > 0);
  queue_.reserve(queue_size);
}

void OdometryHandler::callback(const clt_msgs::CustomOdometryConstPtr &msg) {
  queue_.emplace_back(fromRosCustomMsg(msg));
}

} // namespace pfuclt::sensor::odometry
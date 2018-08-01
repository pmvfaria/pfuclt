//
// Created by glawless on 7/30/18.
//

#include "odometry.hpp"

namespace pfuclt::data::odometry{

OdometryData fromCustomMsg(const clt_msgs::CustomOdometryConstPtr &msg){
  return OdometryData{msg->rot1, msg->rot2, msg->translation};
}

OdometryHandler::OdometryHandler(unsigned long queue_size) {
  ROS_ASSERT(queue_size > 0);
  queue_.reserve(queue_size);
}

void OdometryHandler::callback(const clt_msgs::CustomOdometryConstPtr &msg) {
  queue_.emplace_back(fromCustomMsg(msg));
}
}
//
// Created by glawless on 7/30/18.
//

#include "odometry_data.hpp"

namespace pfuclt::sensor::odometry{

OdometryMeasurement fromRosCustomMsg(const clt_msgs::CustomOdometryConstPtr &msg){
  return OdometryMeasurement{msg->header.stamp, msg->rot1, msg->translation, msg->rot2};
}

} // namespace pfuclt::sensor::odometry
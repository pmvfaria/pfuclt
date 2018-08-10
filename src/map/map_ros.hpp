//
// Created by glawless on 8/8/18.
//

#ifndef PFUCLT_MAP_ROS_HPP
#define PFUCLT_MAP_ROS_HPP

#include "landmark_map.hpp"
#include <ros/ros.h>

namespace pfuclt::map{

std::size_t landmarksFromParameter(LandmarkMap& map, const std::string &param, const ros::NodeHandle &nh);

} // namespace pfuclt::map

#endif //PFUCLT_MAP_ROS_HPP

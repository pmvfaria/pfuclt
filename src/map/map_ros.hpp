//
// Created by glawless on 8/8/18.
//

#ifndef PFUCLT_MAP_ROS_HPP
#define PFUCLT_MAP_ROS_HPP

#include "landmark_map.hpp"
#include <ros/ros.h>

namespace pfuclt::map{

/**
 * @brief Loads landmarks from parameter server to map
 * @param map Instance of class LandmarkMap in which the landmarks will be added
 * @param param The key to be used in the parameter server's dictionary
 * @param nh The NodeHandle to access the parameter server
 * @return Number of landmarks in map
 */
std::size_t landmarksFromParameter(LandmarkMap& map, const std::string &param, const ros::NodeHandle &nh);

} // namespace pfuclt::map

#endif //PFUCLT_MAP_ROS_HPP

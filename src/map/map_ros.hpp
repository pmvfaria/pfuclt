#ifndef PFUCLT_MAP_MAP_ROS_HPP
#define PFUCLT_MAP_MAP_ROS_HPP

#include <string>

#include "ros/ros.h"

#include "landmark_map.hpp"


namespace pfuclt::map {

/**
 * @brief Gets coordinates of walls from parameter server
 * @param dim Vector of doubles that will store the coordinates of the walls
 * @param param The key to be used in the parameter server's dictionary
 * @param nh The NodeHandle to access the parameter server
 */
void wallsFromParameter(std::vector<double>& dim, const std::string& param, const ros::NodeHandle& nh);

/**
 * @brief Loads landmarks from parameter server to map
 * @param map Instance of class LandmarkMap in which the landmarks will be added
 * @param param The key to be used in the parameter server's dictionary
 * @param nh The NodeHandle to access the parameter server
 * @return Number of landmarks in map
 */
int landmarksFromParameter(LandmarkMap& map, const std::string& param, const ros::NodeHandle& nh);

} // namespace pfuclt::map

#endif // PFUCLT_MAP_MAP_ROS_HPP

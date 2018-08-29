//
// Created by glawless on 8/21/18.
//

#ifndef PFUCLT_PFUCLT_HPP
#define PFUCLT_PFUCLT_HPP

#include <ros/ros.h>
#include "../particle/particles.hpp"
#include "map/map_ros.hpp"


namespace pfuclt::algorithm {

class PFUCLT {
 private:
  ros::NodeHandle nh_{""};
  ros::NodeHandle pnh_{"~"};
  std::unique_ptr<::pfuclt::map::LandmarkMap> map_;
  std::unique_ptr<::pfuclt::particle::Particles> particles_;

 private:
  /**
   * @brief Get a map of landmarks from ROS parameter server and place it in map_
   * @details hardcoded /world/landmarks parameter is used
   * @return number of added landmarks
   */
  const std::size_t getLandmarkMap();

  /**
   * @brief Initialize particles using ROS parameter server params
   * @remark If the parameters are not found, initializes particles randomly
   * @return true if particles initialized with parameters, false otherwise
   */
  const bool initializeParticles();

 public:
  PFUCLT() = delete;

  /**
   * @brief Main constructor of PFUCLT
   * @param self_robot_id the robot that the algorithm will run on
   */
  explicit PFUCLT(const uint8_t& self_robot_id);
};

} // namespace pfuclt::algorithm

#endif //PFUCLT_PFUCLT_HPP

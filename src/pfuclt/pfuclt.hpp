//
// Created by glawless on 8/21/18.
//

#ifndef PFUCLT_PFUCLT_HPP
#define PFUCLT_PFUCLT_HPP

#include <ros/ros.h>
#include "../particle/particles.hpp"
#include "map/map_ros.hpp"
#include "robot.hpp"

namespace pfuclt::algorithm {

class PFUCLT {
 private:
  ros::NodeHandle nh_{""};
  ros::NodeHandle pnh_{"~"};

  std::vector<std::unique_ptr<::pfuclt::robot::Robot>> robots_;
  std::unique_ptr<::pfuclt::map::LandmarkMap> map_;
  std::unique_ptr<::pfuclt::particle::Particles> particles_;

  ros::Rate rate_;

  // queue to place odometry and other robot callbacks
  ros::CallbackQueue odometry_cb_queue_;

  // async spinner to allow multi-threading with other robots
  std::unique_ptr<ros::AsyncSpinner> odometry_spinner_;

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
  PFUCLT(const PFUCLT &) = delete; // no copy
  PFUCLT(PFUCLT &&) = delete; // no move
  PFUCLT& operator=(const PFUCLT &) = delete; // no copy assign
  PFUCLT& operator=(PFUCLT &&) = delete; // no move assign

  /**
   * @brief Main constructor of PFUCLT
   * @param self_robot_id the robot that the algorithm will run on
   */
  explicit PFUCLT(const uint& self_robot_id);

  void processOdometryAllRobots();

 public:
  void run();
};

} // namespace pfuclt::algorithm

#endif //PFUCLT_PFUCLT_HPP

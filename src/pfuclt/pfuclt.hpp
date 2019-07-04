//
// Created by glawless on 8/21/18.
//

#ifndef PFUCLT_PFUCLT_HPP
#define PFUCLT_PFUCLT_HPP

#include <parallel/types.h>
#include <functional>

#include <ros/ros.h>
#include "../particle/particles.hpp"
#include "../map/map_ros.hpp"
#include "robot.hpp"
#include "target.hpp"


//#include "../sensor/odometry_data.hpp"

#include "../sensor/landmark_data.hpp"

#include "../thirdparty/CTPL/ctpl_stl.h"

namespace pfuclt::algorithm {

// Alias to the optional use of parallelization from __gnu_parallel
using optional_parallel = std::optional<const __gnu_parallel::_Parallelism>;


/**
  * @brief The Particle filter class - main class of the particle filter algorithm.
  * Creates and initializes the particle filter
  */
class PFUCLT {
 protected:
  ros::NodeHandle nh_{""};
  ros::NodeHandle pnh_{"~"};

  //int num_robots_;

  std::vector<std::unique_ptr<::pfuclt::robot::Robot>> robots_;
  std::vector<std::unique_ptr<::pfuclt::target::Target>> targets_;
  std::unique_ptr<::pfuclt::map::LandmarkMap> map_;
  std::unique_ptr<::pfuclt::particle::Particles> particles_;

  ros::Rate rate_;

  // async spinner to allow multi-threading with other robots
  std::unique_ptr<ros::AsyncSpinner> robot_spinner_;

  // thread pool to parallelize robot methods
  ctpl::thread_pool pool_;

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

  /**
   * @brief Apply function f to every Robot
   * @param f Function to apply to every robot
   * @param tag Optional parallelization tag from __gnu_parallel
   */
  void foreach_robot(std::function<void(std::unique_ptr<::pfuclt::robot::Robot>&)> const& f,
                      const optional_parallel& tag = std::nullopt);
  void foreach_robot(std::function<void(const std::unique_ptr<::pfuclt::robot::Robot>&)> const& f, 
                      const optional_parallel& tag = std::nullopt) const;


 public:
  PFUCLT() = delete;
  PFUCLT(const PFUCLT &) = delete; // no copy
  PFUCLT(PFUCLT &&) = delete; // no move
  PFUCLT& operator=(const PFUCLT &) = delete; // no copy assign
  PFUCLT& operator=(PFUCLT &&) = delete; // no move assign

  uint self_robot_id;

  /**
   * @brief Main constructor of PFUCLT
   * @param self_robot_id The robot that the algorithm will run on
   */
  explicit PFUCLT(const uint self_robot_id);

  void predict();
  void update();

 public:
  void run();

};

} // namespace pfuclt::algorithm

#endif //PFUCLT_PFUCLT_HPP

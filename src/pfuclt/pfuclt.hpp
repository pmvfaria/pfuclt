#ifndef PFUCLT_PFUCLT_PFUCLT_HPP
#define PFUCLT_PFUCLT_PFUCLT_HPP

#include <functional>
#include <optional>
#include <random>
#include <vector>
#include <memory>
#include <parallel/types.h>

#include "ros/ros.h"

#include "pfuclt/robot.hpp"
#include "pfuclt/target.hpp"
#include "pfuclt/state.hpp"
#include "particle/particles.hpp"
#include "map/map_ros.hpp"
#include "ros/pfuclt_publisher.hpp"


namespace pfuclt::algorithm {

// Alias to the optional use of parallelization from __gnu_parallel
using optional_parallel = std::optional<const __gnu_parallel::_Parallelism>;

using generator_type = std::mt19937;

/**
  * @brief The Particle filter class - main class of the particle filter algorithm.
  * Creates and initializes the particle filter
  */
class PFUCLT
{
 public:
  PFUCLT() = delete;
  PFUCLT(const PFUCLT &) = delete; // no copy
  PFUCLT(PFUCLT &&) = delete; // no move
  PFUCLT& operator=(const PFUCLT &) = delete; // no copy assign
  PFUCLT& operator=(PFUCLT &&) = delete; // no move assign

  /**
   * @brief Constructor of PFUCLT
   * @param self_robot_id The robot that the algorithm will run on
   */
  PFUCLT(const int self_robot_id);
  
  int self_robot_id;
  int num_particles, num_robots, num_targets, num_landmarks;

  /**
   * @brief A constant reference to the state of each robot and target is obtained
   */  
  const pfuclt::state::States& getState() const;

  /**
   * @brief Call this method to start executing the algorithm
   */   
  void run();


 private:
  generator_type generator_;

  ros::NodeHandle nh_{""};
  ros::NodeHandle pnh_{"~"};

  ros::Rate rate_;

  ros::Time time_init_;
  uint time_iteration_ms_;

  std::vector<std::unique_ptr<pfuclt::robot::Robot>> robots_;
  std::vector<std::unique_ptr<pfuclt::target::Target>> targets_;
  std::unique_ptr<pfuclt::map::LandmarkMap> map_;
  std::unique_ptr<pfuclt::particle::Particles> particles_;
  std::unique_ptr<pfuclt::state::State> state_;
  
  std::unique_ptr<pfuclt::publisher::PfucltPublisher> publisher_;

  std::vector<pfuclt::particle::WeightSubparticles> weight_components_robots_;
  std::vector<pfuclt::particle::WeightSubparticles> weight_components_targets_;

  // Publisher class has to be able to access PFUCLT data
  friend class pfuclt::publisher::PfucltPublisher;

  /**
   * @brief Updates robot particles using motion model
   * @details The motion model uses odometry measurements to update the particles
   * @return
   */  
  bool predictRobots();

  /**
   * @brief Updates target particles using motion model
   */  
  void predictTargets();

  /**
   * @brief Computes partial weight of the particles (the one related to the robots
   * subparticles) using landmark measurements
   */  
  bool fuseLandmarks();

  /**
   * @brief Computes partial weight of the particles (the one related to the targets
   * subparticles) using target measurements
   */  
  bool fuseTargets();

  /**
   * @brief The partial weights computed in fuseLandmarks() and fuseTargets() are put
   * together, completing the step of weight update of the particle filter
   */  
  void updateWeights();

  /**
   * @brief The resample step of the particle filter
   * @details A modified Multinomial Resampling method is used in this step
   */  
  void resample();

  /**
   * @brief Get covariance coefficients from ROS parameter server
   */
  void getCovariances();

  /**
   * @brief Get a map of landmarks from ROS parameter server and place it in map_
   * @details hardcoded /world/landmarks parameter is used
   * @return number of added landmarks
   */
  int getLandmarkMap();

  /**
   * @brief Initialize particles using ROS parameter server params
   * @remark If the parameters are not found, initializes particles randomly
   * @return true if particles initialized with parameters, false otherwise
   */
  bool initializeParticles();

  /**
   * @brief Apply function f to every Robot
   * @param f Function to apply to every robot
   * @param tag Optional parallelization tag from __gnu_parallel
   */
  void forEachRobot(std::function<void(std::unique_ptr<::pfuclt::robot::Robot>&)> const& f,
                      const optional_parallel& tag = std::nullopt);
  void forEachRobot(std::function<void(const std::unique_ptr<::pfuclt::robot::Robot>&)> const& f, 
                      const optional_parallel& tag = std::nullopt) const;
  void forEachTarget(std::function<void(std::unique_ptr<::pfuclt::target::Target>&)> const& f,
                      const optional_parallel& tag = std::nullopt);
  void forEachTarget(std::function<void(const std::unique_ptr<::pfuclt::target::Target>&)> const& f,
                      const optional_parallel& tag = std::nullopt) const;

};

} // namespace pfuclt::algorithm

#endif // PFUCLT_PFUCLT_PFUCLT_HPP

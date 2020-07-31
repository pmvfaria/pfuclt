#ifndef PFUCLT_PFUCLT_TARGET_HPP
#define PFUCLT_PFUCLT_TARGET_HPP

#include <string>
#include <random>

#include "ros/ros.h"

#include "particle/subparticle.hpp"
#include "particle/particles.hpp"

#include "sensor/measurements_data.hpp"


namespace pfuclt::target {

using generator_type = std::mt19937;

/**
 * @brief The target class - Contains all the information of a specific target, like its
 * id and a pointer to its subparicles set in the particle filter. The target motion model
 * is implemented here.
 * All targets should be instances if this class.
 */
class Target
{
 public:
  // id of this target - they should start at 0
  const int idx;

  // name of this target - should end with a number
  const std::string name;

  // pointer to this target's sub-particles
  pfuclt::particle::TargetSubparticles *subparticles;

  ros::Time last_motion;

  Target() = delete;
  Target(const Target &) = delete; // no copy
  Target(Target &&) = delete; // no move
  Target& operator=(const Target &) = delete; // no copy assign
  Target& operator=(Target &&) = delete; // no move assign

  /**
   * @brief Constructor of Target
   * @param id the id of this target (usually should start at 0)
   * @param subparticles pointer to the subparticles of this target in a set of particles
   */
  Target(const int id, pfuclt::particle::TargetSubparticles* target_subparticles);

  /**
   * @brief Updates subparticles based on a simple motion model (uniform random acceleration)
   */
  void motionModel();

  /**
   * @brief If target is found after being missing, generate new target subparticles based only
   * on the target measurement
   * @details Only the first half of the target subparticles are substituted on the particle set
   */ 
  void observationModel(const particle::RobotSubparticle& robot_subparticle,
                        const sensor::measurement::Measurement& target_measurement);

  /**
   * @brief Computes the next standard deviation to be used in the target predict step of the
   * algorithm.
   * @param weights Weights associated with each particle
   * @details If the weights sum of the first 1/10 particles is smaller than 1E-10, the standard
   * deviation used in predictTargets() is increased for the next target predict step.
   */
  void computeStdDev(const pfuclt::particle::WeightSubparticles& weights);


 private:
  std::random_device rd_{};
  generator_type generator_;

  static constexpr auto name_prefix_ = "target";

  // Standard Deviation loaded from parameter server
  double param_stddev_;

  double motion_mean_;
  double motion_stddev_;

  /**
   * @brief Loads the motion standard deviation from the parameter server and initializes
   * the variable last_motion to the current time
   * @details This method is called in the constructor
   */  
  void initialize();
};

} // namespace pfuclt::target

#endif // PFUCLT_PFUCLT_TARGET_HPP

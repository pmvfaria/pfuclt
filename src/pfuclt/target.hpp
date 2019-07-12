#ifndef PFUCLT_TARGET_HPP
#define PFUCLT_TARGET_HPP

#include <ros/ros.h>

#include <random>
#include <queue>
#include <cmath>

#include "../particle/particles.hpp"

namespace pfuclt::target {

using generator_type = std::mt19937;

/**
 * @brief The target class - Contains all the information of a specific target, like its
 * id or a pointer to its subparicle set in the particle filter. The target motion model
 * is implemented here.
 * All targets should be instances if this class.
 */
class Target {

 public:
  // id of this target - they should start at 0
  const uint idx;

  // name of this target - should end with a number
  const std::string name;

  double motion_mean;
  double motion_stddev;

 private:
  std::random_device rd_{};
  generator_type generator_;

  static constexpr auto name_prefix_ = "target";

  ros::Time last_motion;

  void initialize();

 public:
  // pointer to this target's sub-particles
  particle::TargetSubParticles *subparticles;

  Target() = delete;
  Target(const Target &) = delete; // no copy
  Target(Target &&) = delete; // no move
  Target& operator=(const Target &) = delete; // no copy assign
  Target& operator=(Target &&) = delete; // no move assign

  /**
   * Constructor
   * @param idx the id of this target (usually should start at 0)
   * @param subparticles pointer to the subparticles of this target in a set of particles
   */
  Target(uint id, particle::TargetSubParticles* p_subparticles);

  void motionModel();
};

} // namespace pfuclt::target

#endif //PFUCLT_TARGET_HPP

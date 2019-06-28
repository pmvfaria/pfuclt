#ifndef PFUCLT_TARGET_HPP
#define PFUCLT_TARGET_HPP

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <random>
#include <queue>

#include <clt_msgs/Measurement.h>

#include "../particle/particles.hpp"
#include "../sensor/target_data.hpp"

namespace pfuclt::target {

using generator_type = std::mt19937;


/**
 * @brief The target class - Contains all the information of a specific target, like its id
 * or a pointer to its subparicles set in the particle filter. The target motion model is
 * implemented here.
 * All targets should be instances if this class.
 */
class Target {

 public:
  // id of this target - they should start at 0
  const uint idx;

  // pointer to this target's sub-particles
  particle::TargetSubParticles *subparticles;

 private:
  std::random_device rd_{};
  generator_type generator_;

  void processTargetModel();

 public:
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

};

} // namespace pfuclt::target

#endif //PFUCLT_TARGET_HPP

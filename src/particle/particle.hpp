#ifndef PFUCLT_PARTICLE_H
#define PFUCLT_PARTICLE_H

#include <vector>
#include <ostream>
#include <mutex>
#include <atomic>
#include <parallel/types.h>
#include "subparticle.hpp"

namespace pfuclt::particle {

/*
 * Stores a set of robot, target and weight sub-particles
 * Does not care about target indices, an external class should do the map if needed
 */
class Particles {
 private:
  std::mutex weights_mutex_;

 public:
  std::vector<RobotSubParticles> robots;
  std::vector<TargetSubParticles> targets;
  WeightSubParticles weights;
  std::atomic_uint_fast16_t num_particles;

  // Delete some constructors and operators
  // Default constructor
  Particles() = delete;
  // Copy constructor
  Particles(const Particles&) = delete;
  // Assignment operator
  Particles &operator=(const Particles&) = delete;

  /*
   * Crates a particle set with the options defined by the arguments
   * All particle values are initialized to 0.0, including weights
   */
  Particles(unsigned int num_particles, unsigned int num_robots, unsigned int num_targets);

  /**   \brief Remove target sub-particles by iterator point to it
    *   \arg t iterator pointing to target to remove
    *   \throws std::out_of_range if iterator can't be de-referenced or outside of targets range
    */
  Particles &removeTarget(std::vector<TargetSubParticles>::iterator t);


  /**   \brief Remove target sub-particles by its index
    *   \arg t 0-indexed index of the target to remove
    *   \throws std::out_of_range if target does not exist
    */
  Particles &removeTarget(unsigned int t);

  /*
   * Adds a set of target sub-particles at the back of the targets
   */
  Particles &addTarget();

  /*
   * Normalizes the weights of the specified particle set
   */
  WeightSubParticles &normalizeWeights();

  /*
   * Parallelized version
   */
  WeightSubParticles &normalizeWeights(__gnu_parallel::_Parallelism tag);

};
std::ostream &operator<<(std::ostream &os, const pfuclt::particle::Particles &p);

} // namespace pfuclt::particle

#endif //PFUCLT_PARTICLE_H

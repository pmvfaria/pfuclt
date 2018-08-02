#ifndef PFUCLT_PARTICLE_H
#define PFUCLT_PARTICLE_H

#include <vector>
#include <ostream>
#include <mutex>
#include <atomic>
#include <parallel/types.h>
#include "subparticle.hpp"

namespace pfuclt::particle {

/** \class Particles
  * \brief Stores a set of robot, target and weight sub-particles
  * \details Does not care about target indices, an external class should do the map if needed
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

  /** \brief Creates a particle set with the options defined by the arguments
    * \details All particle values are initialized to 0.0, including weights
    * \param num_particles The number of particles for each sub-particle set
    * \param num_robots The number of robot sub-particle sets
    * \param num_targets The number of target sub-particle sets
    */
  Particles(unsigned int num_particles, unsigned int num_robots, unsigned int num_targets);

  /** \brief Remove target sub-particles by iterator point to it
    * \param t iterator pointing to target to remove
    * \throws std::out_of_range if iterator can't be de-referenced or outside of targets range
    */
  Particles &removeTarget(std::vector<TargetSubParticles>::iterator t);

  /** \brief Remove target sub-particles by its index
    * \param t 0-indexed index of the target to remove
    * \throws std::out_of_range if target does not exist
    */
  Particles &removeTarget(unsigned int t);

  /** \brief Adds a set of target sub-particles at the back of the targets
    * \attention Has noexcept (will terminate if throwing)
    */
  Particles &addTarget() noexcept;

  /** \brief Normalizes the weights, each to 1/num_particles
    * \throws std::range_error if the sum of weights is 0.0
    */
  WeightSubParticles &normalizeWeights();

  /** \brief (Parallelized version) Normalizes the weights, each to 1/num_particles
    * \throws std::range_error if the sum of weights is 0.0
    * \param tag Parallelization tag from __gnu_parallel
    */
  WeightSubParticles &normalizeWeights(__gnu_parallel::_Parallelism tag);

};
std::ostream &operator<<(std::ostream &os, const pfuclt::particle::Particles &p);

} // namespace pfuclt::particle

#endif //PFUCLT_PARTICLE_H

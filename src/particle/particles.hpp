#ifndef PFUCLT_PARTICLES_HPP
#define PFUCLT_PARTICLES_HPP

#include <vector>
#include <ostream>
#include <mutex>
#include <atomic>
#include <parallel/types.h>
#include <functional>
#include "subparticle.hpp"

namespace pfuclt::particle {

/** @class Particles
  * @brief Stores a set of robot, target and weight sub-particles
  * @details Does not care about correspondences, an external class should do the map if needed
  */
class Particles {

  using optional_parallel = std::optional<const __gnu_parallel::_Parallelism>;

 private:
  long seed_{std::chrono::system_clock::now().time_since_epoch().count()};

 public:
  size_t num_particles;
  std::vector<RobotSubParticles> robots;
  std::vector<TargetSubParticles> targets;
  WeightSubParticles weights;

 private:
  /**
   * @brief Normalize a vector [of weights]
   * @param vec the vector to be normalized
   * @return true if able to normalize, false otherwise ( weights all zero )
   */
  const bool normalize__impl(std::vector<double>& vec) const;

 public:
  // Delete some constructors and operators
  Particles() = delete;
  // Disable copying
  Particles(const Particles&) = delete;
  Particles& operator=(const Particles&) = delete;

  // Enable moving
  Particles(Particles &&other) noexcept;

  /** @brief Creates a particle set with the options defined by the arguments
    * @details All particle values are initialized to 0.0, including weights
    * @param num_particles The number of particles for each sub-particle set
    * @param num_robots The number of robot sub-particle sets
    * @param num_targets The number of target sub-particle sets
    */
  Particles(size_t num_particles, size_t num_robots, size_t num_targets);

  /** @brief Initializes the particles from a set of uniform distributions
   * @param robot_dist the distribution for variables of each robot
   * @param target_dist the distribution for variables of each target
   * @details each param is a vector of arrays of 2 values each,
   * * left and right values of the uniform distribution for each variable and each robot )
   */
  Particles& initialize( const std::vector<std::array<double[2], RobotSubParticle::number_states>> &robot_dist,
                         const std::vector<std::array<double[2], TargetSubParticle::number_states>> &target_dist);

  /**
   * @brief Initializes the particles randomly around the field
   * @details Hardcoded to distribute around a field of 10x10 meters and {0,0} center
   */
  Particles& initialize();

  /**
   * @brief Remove target sub-particles by iterator point to it
   * @param t rvalue iterator to target to remove
   * @throws std::out_of_range if iterator can't be de-referenced or outside of targets range
   */
  Particles& removeTarget(const std::vector<TargetSubParticles>::iterator& t);

  /**
   * @brief Remove target sub-particles by its index
   * @param t 0-indexed index of the target to remove
   * @throws std::out_of_range if target does not exist
   */
  Particles& removeTarget(const size_t& t);

  /**
   * @brief Adds a set of target sub-particles at the back of the targets
   * @attention Has noexcept (will terminate if throwing)
   */
  Particles& addTarget() noexcept;

  /**
   * @brief Normalizes the weights, each to 1/num_particles
   * @throws std::range_error if the sum of weights is 0.0
   * @param tag optional parallelization tag from __gnu_parallel
   */
  Particles& normalizeWeights(const optional_parallel& tag = std::nullopt);

  /**
   * @brief Resize all sub-particle sets
   * @details If resizing to a larger number of particles, the new ones are initialized to 0.0
   * @param num_particles The desired number of particles
   * @param tag optional parallelization tag from __gnu_parallel
   */
  Particles& resize(const size_t& num_particles, const optional_parallel& tag = std::nullopt);

  /**
   * @brief Calculate the sum of all particle weights
   * @return Sum of weights
   */
  double sumOfWeights();

  /**
   * @brief Get a copy of the particle weights, normalized
   * @return copy of normalized weights
   */
  WeightSubParticles getNormalizedWeightsCopy() const;

  void foreach_robot(std::function<void(RobotSubParticles&)> const& f, const optional_parallel& tag = std::nullopt);
  void foreach_robot(std::function<void(const RobotSubParticles&)> const& f, const optional_parallel& tag = std::nullopt) const;

  void foreach_target(std::function<void(TargetSubParticles&)> const& f, const optional_parallel& tag = std::nullopt);
  void foreach_target(std::function<void(const TargetSubParticles&)> const& f, const optional_parallel& tag = std::nullopt) const;
};

std::ostream& operator<<(std::ostream &os, const Particles &p);

} // namespace pfuclt::particle

#endif //PFUCLT_PARTICLES_HPP

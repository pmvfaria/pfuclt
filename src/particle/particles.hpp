#ifndef PFUCLT_PARTICLES_PARTICLES_HPP
#define PFUCLT_PARTICLES_PARTICLES_HPP

#include <array>
#include <vector>
#include <ostream>
#include <functional>
#include <chrono>
#include <optional>
#include <parallel/types.h>

#include "subparticle.hpp"


namespace pfuclt::particle {

/** 
 * @class Particles
 * @brief Stores a set of robot, target and weight sub-particles
 * @details Does not care about correspondences, an external class should do the map if needed
 */
class Particles
{
  // Alias to the optional use of parallelization from __gnu_parallel
  using optional_parallel = std::optional<const __gnu_parallel::_Parallelism>;

 public:
  int num_particles;
  std::vector<RobotSubparticles> robots;
  std::vector<TargetSubparticles> targets;
  WeightSubparticles weights;

  Particles() = delete;
  Particles(const Particles&) = delete; // no copy
  Particles& operator=(const Particles&) = delete; // no copy assign

  Particles(Particles&& other) noexcept;  // enable moving
  Particles& operator=(Particles && other) noexcept; // enable move assign

  /**
   * @brief Creates a particle set with the options defined by the arguments
   * @details Weigth particles are initialized to 1.0 / num_particles
   * @param num_particles The number of particles for each sub-particle set
   * @param num_robots The number of robot sub-particle sets
   * @param num_targets The number of target sub-particle sets
   */
  Particles(const int& num_particles, const int& num_robots, const int& num_targets);

  /**
   * @brief Initializes the particles from a set of uniform distributions
   * @param robot_dist the distribution for variables of each robot
   * @param target_dist the distribution for variables of each target
   * @details each param is a vector of arrays of 2 values each, left and right
   * values of the uniform distribution for each variable and each robot)
   * @details If target distribution is not provided, a random one is used
   * (with left value -10 and right value 10)
   */
  Particles& initialize(const std::vector<std::array<double[2], RobotSubparticle::number_states>>& robot_dist,
                        const std::vector<std::array<double[2], TargetSubparticle::number_states>>& target_dist);

  /**
   * @brief Remove target sub-particles by iterator point to it
   * @param t rvalue iterator to target to remove
   * @throw std::out_of_range if iterator can't be de-referenced or outside of targets range
   */
  Particles& removeTarget(const std::vector<TargetSubparticles>::iterator& t);

  /**
   * @brief Remove target sub-particles by its index
   * @param t 0-indexed index of the target to remove
   * @throw std::out_of_range if target does not exist
   */
  Particles& removeTarget(const int& t);

  /**
   * @brief Adds a set of target sub-particles at the back of the targets
   * @attention Has noexcept (will terminate if throwing)
   */
  Particles& addTarget() noexcept;

  /**
   * @brief Normalizes weights
   * @throw std::range_error if the sum of weights is 0.0 or negative
   * @param tag Optional parallelization tag from __gnu_parallel
   */
  Particles& normalizeWeights(const optional_parallel& tag = std::nullopt);

  /**
   * @brief Resize all sub-particle sets
   * @details If resizing to a larger number of particles, the new ones are initialized to 0.0
   * @param num_particles The desired number of particles
   * @param tag Optional parallelization tag from __gnu_parallel
   */
  Particles& resize(const int& num_particles, const optional_parallel& tag = std::nullopt);

  /**
   * @brief Get a copy of the particle weights, normalized
   * @return copy of normalized weights
   */
  WeightSubparticles getNormalizedWeightsCopy() const;

  /**
   * @brief Calculate the sum of all particle weights
   * @return Sum of weights
   */
  double sumOfWeights() const;

  /**
   * @brief Prints weight for each particle
   * @details It will only print if in debug mode
   */
  void printWeights(std::string pre) const;

  /**
   * @brief Apply function f to every robot subparticle set
   * @param f Function to apply to every robot
   * @param tag Optional parallelization tag from __gnu_parallel
   */
  void forEachRobot(std::function<void(RobotSubparticles&)> const& f, const optional_parallel& tag = std::nullopt);
  void forEachRobot(std::function<void(const RobotSubparticles&)> const& f, const optional_parallel& tag = std::nullopt) const;

  /**
   * @brief Apply function f to every target subparticle set
   * @param f Function to apply to every target
   * @param tag Optional parallelization tag from __gnu_parallel
   */
  void forEachTarget(std::function<void(TargetSubparticles&)> const& f, const optional_parallel& tag = std::nullopt);
  void forEachTarget(std::function<void(const TargetSubparticles&)> const& f, const optional_parallel& tag = std::nullopt) const;


 private:
  long seed_{std::chrono::system_clock::now().time_since_epoch().count()};
};

std::ostream& operator<<(std::ostream& os, const Particles& p);

} // namespace pfuclt::particle

#endif // PFUCLT_PARTICLES_PARTICLES_HPP

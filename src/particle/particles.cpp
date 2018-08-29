#include "particles.hpp"
#include <parallel/numeric>
#include <parallel/algorithm>
#include <numeric>
#include <algorithm>
#include <random>
#include <cassert> /* assert - c++ library deprecated */
#include <cmath> /* pi */
#include <iostream>

namespace pfuclt::particle {

Particles::Particles(const size_t num_particles, const size_t num_robots, const size_t num_targets) :
    num_particles(num_particles),
    robots(num_robots, RobotSubParticles(num_particles, RobotSubParticle(0.0))),
    targets(num_targets, TargetSubParticles(num_particles, TargetSubParticle(0.0))),
    weights(num_particles, 0.0) {
  // Nothing to do
}

Particles::Particles(Particles &&other) noexcept :
    num_particles(other.num_particles),
    robots(std::move(other.robots)),
    targets(std::move(other.targets)),
    weights(std::move(other.weights)) {
  // Nothing to do
}

const bool Particles::normalize__impl(std::vector<double>& vec) const {
  const auto sum = std::accumulate(vec.begin(), vec.end(), 0.0);
  if (sum == 0.0) {
    return false;
  }
  else if(sum != 1.0) {
    std::transform(vec.begin(), vec.end(), vec.begin(), [sum](auto &w) { return w / sum; });
  }
  return true;
}

Particles &Particles::initialize(
    const std::vector<std::array<double[2], RobotSubParticle::number_states>> &robot_dist,
    const std::vector<std::array<double[2], TargetSubParticle::number_states>> &target_dist) {

  assert(robot_dist.size() == robots.size() && target_dist.size() == targets.size());

  std::default_random_engine generator(seed_);

  for (uint r = 0; r < robot_dist.size(); ++r) {
    for (uint v = 0; v < RobotSubParticle::number_states; ++v) {
      const auto &params = robot_dist[r][v];

      std::uniform_real_distribution<double> dist(params[0], params[1]);

      for (auto &subparticle: robots[r]) {
        subparticle[v] = dist(generator);
      }
    }
  }

  for (uint t = 0; t < target_dist.size(); ++t) {
    for (uint v = 0; v < TargetSubParticle::number_states; ++v) {
      const auto &params = target_dist[t][v];

      std::uniform_real_distribution<double> dist(params[0], params[1]);

      for (auto &subparticle: targets[t]) {
        subparticle[v] = dist(generator);
      }
    }
  }

  return *this;
}

Particles &Particles::initialize() {

  std::array<double[2], RobotSubParticle::number_states> robot_dist{-10, 10, -10, 10, -M_PI, M_PI};
  std::array<double[2], TargetSubParticle::number_states> target_dist{-10, 10, -10, 10, 0, 5};

  return initialize(std::vector<std::array<double[2], RobotSubParticle::number_states>>(robots.size(), robot_dist),
                    std::vector<std::array<double[2], TargetSubParticle::number_states>>(targets.size(), target_dist));
}

Particles &Particles::removeTarget(const std::vector<TargetSubParticles>::iterator &t) {
  if (targets.empty()) {
    throw std::out_of_range("std::out_of_range exception: targets is empty");
  }

  if (!(t >= targets.begin() && t <= targets.end())) {
    throw std::out_of_range("std::out_of_range exception: specified target does not exist");
  }

  targets.erase(t);
  return *this;
}

Particles &Particles::removeTarget(const size_t &t) {
  return removeTarget(targets.begin() + t);
}

Particles &Particles::addTarget() noexcept {
  targets.emplace_back(TargetSubParticles(num_particles, TargetSubParticle(0.0)));
  return *this;
}

Particles &Particles::normalizeWeights() {

  if(normalize__impl(weights))
    return *this;
  else
    throw std::range_error("Weights have a sum of 0.0, not possible to normalize");
}

Particles &Particles::normalizeWeights(const __gnu_parallel::_Parallelism &tag) {

  const auto sum_weights = __gnu_parallel::accumulate(weights.begin(), weights.end(), 0.0, tag);
  if (sum_weights == 0.0) {
    throw std::range_error("Weights have a sum of 0.0, not possible to normalize");
  }
  else if (sum_weights != 1.0) {
    __gnu_parallel::transform(weights.begin(), weights.end(), weights.begin(),
                              [sum_weights](auto &w) { return w / sum_weights; });
  }
  return *this;
}

Particles &Particles::resize(const size_t &num_particles) {

  __gnu_parallel::for_each(robots.begin(), robots.end(), [num_particles](auto &r) { r.resize(num_particles); });
  __gnu_parallel::for_each(targets.begin(), targets.end(), [num_particles](auto &t) { t.resize(num_particles); });
  weights.resize(num_particles);

  return *this;
}
double Particles::sumOfWeights() {
  return std::accumulate(weights.begin(), weights.end(), 0.0);
}

WeightSubParticles Particles::getNormalizedWeightsCopy() const{
  auto weights_copy = weights;

  normalize__impl(weights_copy);
  return weights_copy;
}

std::ostream &operator<<(std::ostream &os, const Particles &p) {
  return os << "Particle set with " << p.robots.size() << " robot sub-particles, " << p.targets.size()
            << " target sub-particles, with a total of " << p.weights.size() << " particles";
}

} // namespace pfuclt::particle

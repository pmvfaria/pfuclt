#include "particle.hpp"
#include <parallel/numeric>
#include <parallel/algorithm>
#include <numeric>
#include <algorithm>

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

Particles &Particles::removeTarget(const std::vector<TargetSubParticles>::iterator& t) {
  if (targets.empty()) {
    throw std::out_of_range("std::out_of_range exception: targets is empty");
  }

  if( !(t >= targets.begin() && t <= targets.end()) ) {
    throw std::out_of_range("std::out_of_range exception: specified target does not exist");
  }

  targets.erase(t);
  return *this;
}

Particles& Particles::removeTarget(const size_t& t) {
  return removeTarget(targets.begin() + t);
}

Particles& Particles::addTarget() noexcept{
  targets.emplace_back(TargetSubParticles(num_particles, TargetSubParticle(0.0)));
  return *this;
}

Particles& Particles::normalizeWeights() {

  const auto sum_weights = std::accumulate(weights.begin(), weights.end(), 0.0);
  if(sum_weights==0.0){
    throw std::range_error("Weights have a sum of 0.0, not possible to normalize");
  }

  std::transform(weights.begin(), weights.end(), weights.begin(), [sum_weights](auto &w) { return w / sum_weights; });

  return *this;
}

Particles& Particles::normalizeWeights(const __gnu_parallel::_Parallelism& tag) {

  const auto sum_weights = __gnu_parallel::accumulate(weights.begin(), weights.end(), 0.0, tag);
  if(sum_weights==0.0){
    throw std::range_error("Weights have a sum of 0.0, not possible to normalize");
  }
  __gnu_parallel::transform(weights.begin(), weights.end(), weights.begin(),
                            [sum_weights](auto &w) { return w / sum_weights; });

  return *this;
}

Particles& Particles::resize(const size_t& num_particles) {

  __gnu_parallel::for_each(robots.begin(), robots.end(), [num_particles](auto &r){ r.resize(num_particles); });
  __gnu_parallel::for_each(targets.begin(), targets.end(), [num_particles](auto &t){ t.resize(num_particles); });
  weights.resize(num_particles);

  return *this;
}

std::ostream &operator<<(std::ostream &os, const Particles &p){
  return os << "Particle set with " << p.robots.size() << " robot sub-particles, " << p.targets.size()
            << " target sub-particles, with a total of " << p.weights.size() << " particles";
}

} // namespace pfuclt::particle

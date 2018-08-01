#include "particle.hpp"
#include <numeric>

namespace pfuclt::particle {

Particles::Particles(const unsigned int num_particles, const unsigned int num_robots, const unsigned int num_targets) :
    num_particles_(num_particles),
    robots(num_robots, RobotSubParticles(num_particles, RobotSubParticle(0.0))),
    targets(num_targets, TargetSubParticles(num_particles, TargetSubParticle(0.0))),
    weights(num_particles, 0.0) {
  // Nothing to do
}

Particles &Particles::removeTarget(const std::vector<TargetSubParticles>::iterator t) {
  if (targets.empty()) {
    throw std::out_of_range("std::out_of_range exception: targets is empty");
  }

  if( !(t >= targets.begin() && t <= targets.end()) ) {
    throw std::out_of_range("std::out_of_range exception: specified target does not exist");
  }

  targets.erase(t);
  return *this;
}

Particles &Particles::removeTarget(const unsigned int t) {
  return removeTarget(targets.begin() + t);
}

Particles &Particles::addTarget() {
  targets.emplace_back(TargetSubParticles(num_particles_, TargetSubParticle(0.0)));
  return *this;
}

WeightSubParticles &Particles::normalizeWeights() {
  //std::lock_guard<std::mutex> guard(weights_mutex_);
  //std::accumulate(weights.begin(), weights.end(), 0.0);

  std::for_each(weights.begin(), weights.end(), []( double& w ) { w+=1.0; });
  //__gnu_parallel::for_each(weights.begin(), weights.end(), []( double& w ) { w+=1.0; }, tag);

  return weights;
}

WeightSubParticles &Particles::normalizeWeights(const __gnu_parallel::_Parallelism tag) {
  //std::lock_guard<std::mutex> guard(weights_mutex_);
  //std::accumulate(weights.begin(), weights.end(), 0.0);

  //std::for_each(weights.begin(), weights.end(), []( double& w ) { w+=1.0; });
  __gnu_parallel::for_each(weights.begin(), weights.end(), []( double& w ) { w+=1.0; }, tag);

  return weights;
}

std::ostream &operator<<(std::ostream &os, const pfuclt::particle::Particles &p){
  return os << "Particle set with " << p.robots.size() << " robot sub-particles, " << p.targets.size()
            << " target sub-particles, with a total of " << p.weights.size() << " particles";
}

} // namespace pfuclt::particle

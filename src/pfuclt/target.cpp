#include "target.hpp"
#include <random>

namespace pfuclt::target {

Target::Target(const uint id, particle::TargetSubParticles* p_subparticles)
  : idx(id), name(Target::name_prefix_ + std::to_string(idx+1)), 
    generator_(rd_()), subparticles(p_subparticles) { }


void Target::processTargetModel() {

  // Target motion model
  std::normal_distribution<double> targetAccel(TARGET_MEAN, TARGET_STDDEV);

  for(auto& particle: *subparticles) {

    // Update target subparticles
    particle.x += 0.5 * targetAccel(generator_) * pow(timestamp, 2);
    particle.y += 0.5 * targetAccel(generator_) * pow(timestamp, 2);
    particle.z += 0.5 * targetAccel(generator_) * pow(timestamp, 2);
  }
  
}

} // namespace pfuclt::target
#include "target.hpp"
#include <random>

namespace pfuclt::target {

Target::Target(const uint id, particle::TargetSubParticles* p_subparticles)
  : idx(id), name(Target::name_prefix_ + std::to_string(idx+1)), 
    generator_(rd_()), subparticles(p_subparticles) {

  this->initialize();
}


void Target::motionModel() {

  // Target motion model
  std::normal_distribution<double> targetAccel(motion_mean, motion_stddev);

  // TODO refactor and abstract a better motion model

  auto now = ros::Time::now();
  auto diff_sec = (now - last_motion).toSec();

  for(auto& particle: *subparticles) {

    // Update target subparticles
    particle.x += 0.5 * targetAccel(generator_) * pow(diff_sec, 2);
    particle.y += 0.5 * targetAccel(generator_) * pow(diff_sec, 2);
    particle.z += 0.5 * targetAccel(generator_) * pow(diff_sec, 2);
  }

  last_motion = now;
}

} // namespace pfuclt::target
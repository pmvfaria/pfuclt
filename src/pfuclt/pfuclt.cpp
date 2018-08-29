//
// Created by glawless on 8/21/18.
//

#include "pfuclt.hpp"

namespace pfuclt::algorithm {

PFUCLT::PFUCLT(const uint8_t &self_robot_id) {

  // Get landmarks map
  const auto num_landmarks = this->getLandmarkMap();
  ROS_ASSERT(num_landmarks > 0);

  ROS_INFO_STREAM("Added " << num_landmarks << " landmarks");
  ROS_INFO_STREAM(*map_);

  // Create and initialize particles
  int num_particles, num_robots, num_targets;
  ROS_ASSERT_MSG(pnh_.getParam("particles", num_particles), "Parameter num_particles is required");
  ROS_ASSERT_MSG(nh_.getParam("num_robots", num_robots), "Parameter num_robots is required");
  ROS_ASSERT_MSG(nh_.getParam("num_targets", num_targets), "Parameter num_targets is required");

  particles_ = std::make_unique<::pfuclt::particle::Particles>(num_particles, num_robots, num_targets);

  auto initializedParticlesFromParameter = this->initializeParticles();
  if (initializedParticlesFromParameter)
    ROS_INFO("Initialized particles using parameters");
  else
    ROS_INFO("Initialized particles randomly");

  ROS_INFO_STREAM(*particles_);
}

} // namespace pfuclt::algorithm

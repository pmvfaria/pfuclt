//
// Created by glawless on 8/21/18.
//

#include <future>

#include "pfuclt.hpp"
#include <boost/thread.hpp>
#include "../thirdparty/stopwatch/StopWatch.h"

namespace pfuclt::algorithm {

PFUCLT::PFUCLT(const uint self_robot_id)
  : rate_(50), pool_(1), self_robot_id(self_robot_id) {

  // Get important parameters
  int num_particles, num_robots_, num_targets;
  ROS_ASSERT_MSG(pnh_.getParam("particles", num_particles), "Parameter num_particles is required");
  ROS_ASSERT_MSG(nh_.getParam("num_robots_", num_robots_), "Parameter num_robots_ is required");
  ROS_ASSERT_MSG(nh_.getParam("num_targets", num_targets), "Parameter num_targets is required");

  // Get landmarks map
  const auto num_landmarks = this->getLandmarkMap();
  ROS_ASSERT(num_landmarks > 0);

  ROS_INFO_STREAM("Added " << num_landmarks << " landmarks");
  ROS_INFO_STREAM(*map_);

  // Create and initialize particles
  particles_ = std::make_unique<::pfuclt::particle::Particles>(num_particles, num_robots_, num_targets);

  auto initializedParticlesFromParameter = this->initializeParticles();
  if (initializedParticlesFromParameter)
    ROS_INFO("Initialized particles using parameters");
  else
    ROS_INFO("Initialized particles randomly");

  ROS_INFO_STREAM(*particles_);

  // This spinner will help multi-thread robot odometry processing, which is independent of other robots
  robot_spinner_ = std::make_unique<ros::AsyncSpinner>(num_robots_);

  pool_.resize(num_robots_);

  // Create robots
  robots_.reserve((size_t)num_robots_);
  for (uint r = 0; r < (uint) num_robots_; ++r) {
    robots_.emplace_back(std::make_unique<::pfuclt::robot::Robot>(r, &particles_->robots[r]));
    ROS_INFO_STREAM("Robot created with index " << robots_[r]->idx << " and name " << robots_[r]->name);
  }
}

void PFUCLT::predict()
{
  std::vector<std::future<void>> results;
  for (auto &robot: robots_) 
  {
    results.emplace_back(pool_.push(
      [&robot](int id){robot->predict();}
    ));
  }
  for (auto &result: results) {
    result.get();
  }
}

void PFUCLT::run() {
  robot_spinner_->start();

  while(ros::ok()) {
    robot_spinner_->stop();
    predict();


    robot_spinner_->start();
    rate_.sleep();
  }

  robot_spinner_->stop();
}

} // namespace pfuclt::algorithm

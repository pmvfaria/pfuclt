//
// Created by glawless on 8/21/18.
//

#include "pfuclt.hpp"
#include <boost/thread.hpp>
#include "thirdparty/stopwatch/StopWatch.h"

namespace pfuclt::algorithm {

PFUCLT::PFUCLT(const uint &self_robot_id)
  : rate_(50) {

  // Get important parameters
  int num_particles, num_robots, num_targets;
  ROS_ASSERT_MSG(pnh_.getParam("particles", num_particles), "Parameter num_particles is required");
  ROS_ASSERT_MSG(nh_.getParam("num_robots", num_robots), "Parameter num_robots is required");
  ROS_ASSERT_MSG(nh_.getParam("num_targets", num_targets), "Parameter num_targets is required");

  // Get landmarks map
  const auto num_landmarks = this->getLandmarkMap();
  ROS_ASSERT(num_landmarks > 0);

  ROS_INFO_STREAM("Added " << num_landmarks << " landmarks");
  ROS_INFO_STREAM(*map_);

  // Create and initialize particles
  particles_ = std::make_unique<::pfuclt::particle::Particles>(num_particles, num_robots, num_targets);

  auto initializedParticlesFromParameter = this->initializeParticles();
  if (initializedParticlesFromParameter)
    ROS_INFO("Initialized particles using parameters");
  else
    ROS_INFO("Initialized particles randomly");

  ROS_INFO_STREAM(*particles_);

  // This spinner will help multi-thread robot odometry processing, which is independent of other robots
  odometry_spinner_ = std::make_unique<ros::AsyncSpinner>(num_robots, &odometry_cb_queue_);

  // Create robots
  robots_.reserve((size_t)num_robots);
  for (uint r = 0; r < (uint) num_robots; ++r) {
    robots_.emplace_back(std::make_unique<::pfuclt::robot::Robot>(r, &particles_->robots[r], &odometry_cb_queue_));
    ROS_INFO_STREAM("Robot created with index " << robots_[r]->idx << " and name " << robots_[r]->name);
  }
}

void PFUCLT::run() {

  while(ros::ok()) {
    processOdometryAllRobots();

    // Start spinning all robots so that each get their odometry during the sleep
    odometry_spinner_->start();

    rate_.sleep();
  }
}

void PFUCLT::processOdometryAllRobots() {

  ros::Rate r(1000);

  //TODO remove
  StopWatch watch;

  // Wait until the queue is empty (no more odometry to process)
  while(ros::ok() && !odometry_cb_queue_.isEmpty())
    r.sleep();
  odometry_spinner_->stop();

  //TODO it might be better to only process odometry up to the latest landmark msg time, or the weights will be wrong because it's not synced

  ROS_WARN_STREAM("This loop took" << watch.ElapsedMs() << " ms");
}

} // namespace pfuclt::algorithm

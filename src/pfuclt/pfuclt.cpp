//
// Created by glawless on 8/21/18.
//

#include <future>
#include <parallel/algorithm>

#include "pfuclt.hpp"
#include <boost/thread.hpp>
#include "../thirdparty/stopwatch/StopWatch.h"


namespace pfuclt::algorithm {

PFUCLT::PFUCLT(const uint self_robot_id)
  : rate_(50), pool_(1), self_robot_id(self_robot_id) {

  // Get important parameters
  ROS_ASSERT_MSG(pnh_.getParam("particles", num_particles), "Parameter num_particles is required");
  ROS_ASSERT_MSG(nh_.getParam("num_robots", num_robots), "Parameter num_robots is required");
  ROS_ASSERT_MSG(nh_.getParam("num_targets", num_targets), "Parameter num_targets is required");
  ROS_ASSERT_MSG(nh_.getParam("landmark_K_range", sensor::landmark::K_range), "Parameter landmark_K_range is required");
  ROS_ASSERT_MSG(sensor::landmark::K_range > 0, "Parameter landmark_K_range must be larger than 0");
  ROS_ASSERT_MSG(nh_.getParam("landmark_K_bearing", sensor::landmark::K_bearing), "Parameter landmkar_K_bearing is required");
  ROS_ASSERT_MSG(sensor::landmark::K_bearing > 0, "Parameter landmark_K_bearing must be larger than 0");

  // Get landmarks map
  num_landmarks = this->getLandmarkMap();
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
  robot_spinner_ = std::make_unique<ros::AsyncSpinner>(num_robots);

  pool_.resize(num_robots);

  // Create robots
  robots_.reserve((size_t)num_robots);
  for (uint r = 0; r < (uint) num_robots; ++r) {
    robots_.emplace_back(std::make_unique<::pfuclt::robot::Robot>(r, &particles_->robots[r], map_.get()));
    ROS_INFO_STREAM("Robot created with index " << robots_[r]->idx << " and name " << robots_[r]->name);
  }
}

void PFUCLT::foreach_robot(std::function<void(std::unique_ptr<::pfuclt::robot::Robot>&)> const& f, const optional_parallel& tag) {
  if(tag)
    __gnu_parallel::for_each(robots_.begin(), robots_.end(), f, tag.value());
  else
    std::for_each(robots_.begin(), robots_.end(), f);
}

void PFUCLT::foreach_robot(std::function<void(const std::unique_ptr<::pfuclt::robot::Robot>&)> const& f, const optional_parallel& tag) const {
  if(tag)
    __gnu_parallel::for_each(robots_.begin(), robots_.end(), f, tag.value());
  else
    std::for_each(robots_.begin(), robots_.end(), f);
}

void PFUCLT::predictRobots()
{
  this->foreach_robot([] (auto &robot) -> void {
      robot->motionModel();
    },
    __gnu_parallel::parallel_unbalanced);
}

void PFUCLT::predictTargets()
{

}

void PFUCLT::fuseLandmarks()
{

}

void PFUCLT::fuseTargets()
{
  std::vector<particle::WeightSubParticles> probabilities(num_robots, particle::WeightSubParticles(num_particles));
  this->foreach_robot([&probabilities, r=0] (auto &robot) mutable -> void {
      auto landmarks_seen = robot->landmarksUpdate(probabilities[r]);
      if (landmarks_seen > 0)
      {
        robot->clearLandmarkMeasurements();
      }
    },
    __gnu_parallel::parallel_unbalanced);
}

void PFUCLT::run() {
  robot_spinner_->start();

  while(ros::ok()) {
    robot_spinner_->stop();

    predictRobots();
    predictTargets();
    fuseLandmarks();
    fuseTargets();

    robot_spinner_->start();
    rate_.sleep();
  }

  robot_spinner_->stop();
}

} // namespace pfuclt::algorithm

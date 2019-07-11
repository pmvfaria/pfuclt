//
// Created by glawless on 8/21/18.
//

#include <future>
#include <parallel/algorithm> // parallel for_each
#include <numeric> // iota
#include <algorithm> // sort

#include "pfuclt.hpp"
#include <boost/thread.hpp>
#include "../thirdparty/stopwatch/StopWatch.h"


namespace pfuclt::algorithm {

PFUCLT::PFUCLT(const uint self_robot_id)
  : rate_(50), self_robot_id(self_robot_id) {

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

  // Initialize per-robot per-particle persistent weight components
  weight_components_.assign(num_robots, particle::WeightSubParticles(num_particles, 1.0));

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

  // Create robots
  robots_.reserve((size_t)num_robots);
  for (uint r = 0; r < (uint) num_robots; ++r) {
    robots_.emplace_back(std::make_unique<::pfuclt::robot::Robot>(r, &particles_->robots[r], map_.get()));
    ROS_INFO_STREAM("Robot created with index " << robots_[r]->idx << " and name " << robots_[r]->name);
  }

  // Create state
  state_ = std::make_unique<::pfuclt::state::State>(num_robots, num_targets);

  // Create publisher class
  publisher_ = std::make_unique<::pfuclt::publisher::PFUCLTPublisher>(*this);
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
  /**
   * Per-robot weight components are persistent accross iterations
   * In case a robot does not see any landmark, its weight component is kept
   * TODO is this a good approach? See http://dante.isr.tecnico.ulisboa.pt/pf_coop_perception/pfuclt/issues/7
   */

  this->foreach_robot([this] (auto &robot) -> void {
      auto& robot_weights = this->weight_components_[robot->idx];
      auto landmarks_seen = robot->landmarksUpdate(robot_weights);
      if (landmarks_seen > 0)
      {
        robot->clearLandmarkMeasurements();

        // Get sorted weights indices
        std::vector<size_t> idx(robot_weights.size());
        std::iota(idx.begin(), idx.end(), 0);
        std::sort(idx.begin(), idx.end(),
          [&robot_weights](auto i1, auto i2) {return robot_weights[i1] < robot_weights[i2];});

        // Sorting according to indices is O(n) using a O(n) copy of the subparticles, same for weights
        auto& robot_subparticles = *robot->subparticles;
        const auto weights_copy = robot_weights;
        const auto subparticles_copy = robot_subparticles;
        for (std::size_t p{0}, sz{weights_copy.size()}; p<sz; ++p)
        {
          robot_subparticles[p] = subparticles_copy[idx[p]];
          robot_weights[p] = weights_copy[idx[p]];
        }
      }
    },
    __gnu_parallel::parallel_unbalanced);


  // accumulate does not work as our map is robot -> particle weights and not the inverse
  // O(NM)
  for (auto p{0}; p<num_particles; ++p)
  {
    particles_->weights[p] = 1.0;
    for (auto r{0}; r<num_robots; ++r)
    {
      particles_->weights[p] *= weight_components_[r][p];
    }
  }
}

void PFUCLT::fuseTargets()
{

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

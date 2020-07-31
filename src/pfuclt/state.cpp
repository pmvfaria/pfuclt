#include "state.hpp"

#include <numeric>
#include <algorithm>
#include <cmath>
#include <iterator>

namespace pfuclt::state {

using namespace pfuclt::particle;

State::State(pfuclt::particle::Particles* particles)
  : targets_visibility(particles->targets.size(), std::vector<bool>(particles->robots.size(), false)),
    targets_found(particles->targets.size(), -1), particles_(particles)
{
  states.robots.resize(particles_->robots.size());
  states.targets.resize(particles_->targets.size());

  for (RobotSubparticle& robot : states.robots)
    for (int s = 0; s < robot.number_states; ++s)
      robot[s] = 0.0;

  for (TargetSubparticle& target : states.targets)
    for (int s = 0; s < target.number_states; ++s)
      target[s] = 0.0;
}

void State::estimateAvg()
{
  int r = 0;
  particles_->forEachRobot([&r, this] (const RobotSubparticles& robot_subparticles) -> void
  {
    auto& robot_state = this->states.robots[r];

    robot_state.x = std::accumulate(robot_subparticles.begin(), robot_subparticles.end(), 0.0,
                                   [](double sum, const RobotSubparticle& subparticle) {
      return sum + subparticle.x;
    }) / robot_subparticles.size();

    robot_state.y = std::accumulate(robot_subparticles.begin(), robot_subparticles.end(), 0.0,
                                   [](double sum, const RobotSubparticle& subparticle) {
      return sum + subparticle.y;
    }) / robot_subparticles.size();

    robot_state.z = std::accumulate(robot_subparticles.begin(), robot_subparticles.end(), 0.0,
                                   [](double sum, const RobotSubparticle& subparticle) {
      return sum + subparticle.z;
    }) / robot_subparticles.size();

    // Yaw as mean of circular quantities
    double sum_yaw_cos{0.0}, sum_yaw_sin{0.0};
    for (const RobotSubparticle& subparticle : robot_subparticles) {
      sum_yaw_cos += cos(subparticle.yaw);
      sum_yaw_sin += sin(subparticle.yaw);
    }

    // Convert back to polar
    robot_state.yaw = std::atan2(sum_yaw_sin / robot_subparticles.size(),
                                   sum_yaw_cos / robot_subparticles.size());

    ++r;
  });

  int t = 0;
  particles_->forEachTarget([&t, this] (const TargetSubparticles &target_subparticles) -> void
  {
    auto& target_state = this->states.targets[t];

    target_state.x = std::accumulate(target_subparticles.begin(), target_subparticles.end(), 0.0,
                                    [](double sum, const TargetSubparticle& subparticle) {
      return sum + subparticle.x;
    }) / target_subparticles.size();

    target_state.y = std::accumulate(target_subparticles.begin(), target_subparticles.end(), 0.0,
                                    [](double sum, const TargetSubparticle& subparticle) {
      return sum + subparticle.y;
    }) / target_subparticles.size();

    target_state.z = std::accumulate(target_subparticles.begin(), target_subparticles.end(), 0.0,
                                    [](double sum, const TargetSubparticle& subparticle) {
      return sum + subparticle.z;
    }) / target_subparticles.size();

    ++t;
  });
}


void State::estimateWeightedAvg()
{
  const auto normalized_weights = particles_->getNormalizedWeightsCopy();
  
  int r = 0;
  particles_->forEachRobot([&, this](const RobotSubparticles& robot_subparticles) mutable -> void
  {
    auto& robot_state = this->states.robots[r];

    // Reset to zeros
    for (int s = 0; s < robot_state.number_states; s++)
      robot_state[s] = 0.0;

    // Yaw as mean of circular quantities
    double sum_yaw_cos{0.0}, sum_yaw_sin{0.0};

    int idx = 0;
    std::for_each(robot_subparticles.begin(), robot_subparticles.end(),
                  [&](const RobotSubparticle& subparticle) mutable -> void
    {
      const auto& weight = normalized_weights[idx];
      if (weight != 0.0) {
        robot_state.x += subparticle.x * weight;
        robot_state.y += subparticle.y * weight;
        robot_state.z += subparticle.z * weight;
        sum_yaw_cos += std::cos(subparticle.yaw) * weight;
        sum_yaw_sin += std::sin(subparticle.yaw) * weight;
      }

      ++idx;
    });

    // Convert back to polar
    robot_state.yaw = std::atan2(sum_yaw_sin, sum_yaw_cos);

    ++r;
    
  }, __gnu_parallel::parallel_unbalanced);

  int t = 0;
  particles_->forEachTarget([&, this](const TargetSubparticles& target_subparticles) mutable -> void
  {
    auto& target_state = this->states.targets[t];

    // Reset to zeros
    for (int s = 0; s < target_state.number_states; s++)
      target_state[s] = 0.0;

    int idx = 0;
    std::for_each(target_subparticles.begin(), target_subparticles.end(),
                  [&](const TargetSubparticle& subparticle) mutable -> void
    {
      const auto& weight = normalized_weights[idx];
      if (weight != 0.0) {
        target_state.x += subparticle.x * weight;
        target_state.y += subparticle.y * weight;
        target_state.z += subparticle.z * weight;
      }
      ++idx;
    });

    ++t;

  }, __gnu_parallel::parallel_unbalanced);
}


void State::estimateMaxWeight()
{
  // Get index of max weight particle
  const auto idx = std::distance(particles_->weights.begin(),
                                 std::max_element(particles_->weights.begin(), particles_->weights.end()));

  for (size_t r = 0; r < states.robots.size(); ++r)
    states.robots[r] = particles_->robots[r][idx];

  for (size_t t = 0; t < states.targets.size(); ++t)
    states.targets[t] = particles_->targets[t][idx];
}

} // namespace pfuclt::state

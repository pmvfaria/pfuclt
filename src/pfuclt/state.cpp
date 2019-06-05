//
// Created by glawless on 8/27/18.
//

#include "state.hpp"
#include <exception>
#include <numeric>
#include <algorithm>
#include <cmath>

namespace pfuclt::state {

State::State(const size_t num_robots, const size_t num_targets) : robots(num_robots), targets(num_targets) {
  // Nothing to do
}

void _estimate_avg_impl(const Particles &particles, State &state) {

  //TODO check this actually cycles through the robots (r, r+1, ...)
  particles.foreach_robot([r=0, &state] (const auto &p_r) mutable -> void {
    auto &robot_state = state.robots[r];
    ++r;

    robot_state.x = std::accumulate(p_r.begin(), p_r.end(), 0.0, [&](double sum, const RobotSubParticle &rsp) {
      return sum + rsp.x;
    }) / p_r.size();

    robot_state.y = std::accumulate(p_r.begin(), p_r.end(), 0.0, [&](double sum, const RobotSubParticle &rsp) {
      return sum + rsp.y;
    }) / p_r.size();

    // Theta as mean of circular quantities
    double sum_theta_cos{0.0}, sum_theta_sin{0.0};
    for (const RobotSubParticle &rsp: p_r) {
      sum_theta_cos += cos(rsp.theta);
      sum_theta_sin += sin(rsp.theta);
    }

    // Convert back to polar
    robot_state.theta = atan2(sum_theta_sin / p_r.size(), sum_theta_cos / p_r.size());
  });

  particles.foreach_target([t=0, &state] (const TargetSubParticles &p_t) mutable -> void {

    auto &target_state = state.targets[t];
    ++t;

    target_state.y = std::accumulate(p_t.begin(), p_t.end(), 0.0, [](double sum, const TargetSubParticle &tsp) {
      return sum + tsp.x;
    }) / p_t.size();

    target_state.y = std::accumulate(p_t.begin(), p_t.end(), 0.0, [](double sum, const TargetSubParticle &tsp) {
      return sum + tsp.y;
    }) / p_t.size();

    target_state.z = std::accumulate(p_t.begin(), p_t.end(), 0.0, [](double sum, const TargetSubParticle &tsp) {
      return sum + tsp.z;
    }) / p_t.size();
  });
}

void _estimate_weighted_avg_impl(const Particles &particles, State &state) {
  const auto normalized_weights = particles.getNormalizedWeightsCopy();

  particles.foreach_robot([&, r=0] (const RobotSubParticles& p_r) mutable -> void {

    auto &robot_state = state.robots[r];
    ++r;

    // Theta as mean of circular quantities
    double sum_theta_cos{0.0}, sum_theta_sin{0.0};

    std::for_each(p_r.begin(), p_r.end(), [&, idx=0] (const auto &rsp) mutable -> void {
      const auto weight = normalized_weights[idx];
      if (weight != 0.0) {
        robot_state.x += rsp.x * weight;
        robot_state.y += rsp.y * weight;
        sum_theta_cos += cos(rsp.theta) * weight;
        sum_theta_sin += sin(rsp.theta) * weight;
      }
    });

    // Convert back to polar
    state.robots[r].theta = atan2(sum_theta_sin, sum_theta_cos);
  });

  particles.foreach_target([&, t=0] (const TargetSubParticles& p_t) mutable -> void {

    auto &target_state = state.targets[t];
    ++t;

    std::for_each(p_t.begin(), p_t.end(), [&, idx=0] (const auto &tsp) mutable -> void {

      const auto weight = normalized_weights[idx];
      if(weight != 0.0) {
        target_state.x += tsp.x * weight;
        target_state.y += tsp.y * weight;
        target_state.z += tsp.z * weight;
      }
      ++idx;
    });
  });
}

void _estimate_max_weight_impl(const Particles &particles, State &state) {

  // Get index of max weight particle
  const auto idx = std::distance(particles.weights.begin(), std::max_element(particles.weights.begin(), particles.weights.end()));

  for(uint r=0; r < state.robots.size(); ++r) {
    state.robots[r] = particles.robots[r][idx];
  }

  for(uint t=0; t < state.targets.size(); ++t) {
    state.targets[t] = particles.targets[t][idx];
  }
}

State estimateState(const Particles &particles, const StateCalculation opt) {

  State state(particles.robots.size(), particles.targets.size());

  switch(opt) {
    case StateCalculation::AVERAGE : _estimate_avg_impl(particles, state); break;
    case StateCalculation::WEIGHTED_AVERAGE : _estimate_weighted_avg_impl(particles, state); break;
    case StateCalculation::MAX_WEIGHT : _estimate_max_weight_impl(particles, state); break;
  }

  return state;
}
} // namespace pfuclt::state

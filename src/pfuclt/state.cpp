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

  for(uint r=0; r < state.robots.size(); ++r) {
    const auto &p_r = particles.robots[r];
    state.robots[r].y() = std::accumulate(p_r.begin(), p_r.end(), 0.0, [&](double sum, const RobotSubParticle &rsp) {
      return sum + rsp.x();
    }) / p_r.size();

    state.robots[r].y() = std::accumulate(p_r.begin(), p_r.end(), 0.0, [&](double sum, const RobotSubParticle &rsp) {
      return sum + rsp.y();
    }) / p_r.size();

    // Theta as mean of circular quantities
    double sum_theta_cos{0.0}, sum_theta_sin{0.0};
    for (const RobotSubParticle &rsp: p_r) {
      sum_theta_cos += cos(rsp.theta());
      sum_theta_sin += sin(rsp.theta());
    }

    // Convert back to polar
    state.robots[r].theta() = atan2(sum_theta_sin / p_r.size(), sum_theta_cos / p_r.size());
  }

  for(uint t=0; t < state.targets.size(); ++t) {
    const auto &p_t = particles.targets[t];
    state.targets[t].y() = std::accumulate(p_t.begin(), p_t.end(), 0.0, [&](double sum, const TargetSubParticle &tsp) {
      return sum + tsp.x();
    }) / p_t.size();

    state.targets[t].y() = std::accumulate(p_t.begin(), p_t.end(), 0.0, [&](double sum, const TargetSubParticle &tsp) {
      return sum + tsp.y();
    }) / p_t.size();

    state.targets[t].z() = std::accumulate(p_t.begin(), p_t.end(), 0.0, [&](double sum, const TargetSubParticle &tsp) {
      return sum + tsp.z();
    }) / p_t.size();
  }
}

void _estimate_weighted_avg_impl(const Particles &particles, State &state) {
  const auto normalized_weights = particles.getNormalizedWeightsCopy();

  for(uint r=0; r < state.robots.size(); ++r) {
    const auto &p_r = particles.robots[r];

    // Theta as mean of circular quantities
    double sum_theta_cos{0.0}, sum_theta_sin{0.0};
    std::size_t idx=0;

    for (const RobotSubParticle &rsp: p_r) {
      const auto weight = normalized_weights[idx];
      if(weight != 0.0) {
        state.robots[r].x() += rsp.x() * weight;
        state.robots[r].y() += rsp.y() * weight;
        sum_theta_cos += cos(rsp.theta()) * weight;
        sum_theta_sin += sin(rsp.theta()) * weight;
      }
      ++idx;
    }

    // Convert back to polar
    state.robots[r].theta() = atan2(sum_theta_sin, sum_theta_cos);
  }

  for(uint t=0; t < state.targets.size(); ++t) {
    const auto &p_t = particles.targets[t];
    std::size_t idx=0;

    for (const TargetSubParticle &tsp: p_t) {
      const auto weight = normalized_weights[idx];
      if(weight != 0.0) {
        state.targets[t].x() += tsp.x() * weight;
        state.targets[t].y() += tsp.y() * weight;
        state.targets[t].z() += tsp.z() * weight;
      }
      ++idx;
    }
  }
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

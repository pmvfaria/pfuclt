//
// Created by glawless on 8/27/18.
//

#ifndef PFUCLT_STATE_HPP
#define PFUCLT_STATE_HPP

#include "particle/subparticle.hpp"
#include "../particle/particles.hpp"
#include <functional>

namespace pfuclt::state {

using namespace ::pfuclt::particle;
typedef RobotSubParticle RobotState;
typedef TargetSubParticle TargetState;

enum class StateCalculation{ AVERAGE, WEIGHTED_AVERAGE, MAX_WEIGHT };

struct State {
  std::vector<RobotState> robots;
  std::vector<TargetState> targets;

  State(size_t num_robots, size_t num_targets);
};

void _estimate_avg_impl(const Particles& particles, State& state);
void _estimate_weighted_avg_impl(const Particles& particles, State& state);
void _estimate_max_weight_impl(const Particles& particles, State& state);

State estimateState(const Particles& particles, const StateCalculation opt);

} // namespace pfuclt::state

#endif //PFUCLT_STATE_HPP

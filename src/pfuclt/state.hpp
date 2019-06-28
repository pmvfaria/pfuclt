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

/**
 * @brief Estimates the average of each subparticle set
 * @param particles Reference to an intance of the class Particles
 * @param state Reference to an intance of the struct State which will store
 * the average of each subparticle set (excluding for weight subparticle set)
 */
void _estimate_avg_impl(const Particles& particles, State& state);

/**
 * @brief Estimates the weighted average of each subparticle set
 * @param particles Reference to an intance of the class Particles
 * @param state Reference to an intance of the struct State which will store
 * the average of each subparticle set (excluding for weight subparticle set)
 */
void _estimate_weighted_avg_impl(const Particles& particles, State& state);

/**
 * @brief Retrieves the particle with maximum weigth
 * @param particles Reference to an intance of the class Particles
 * @param state Reference to an intance of the struct State which will store
 * the particle with maximum weight (excluding weight subparticle)
 */
void _estimate_max_weight_impl(const Particles& particles, State& state);

/**
 * @brief Choose which state estimate should be calculted
 * @param particles Reference to an intance of the class Particles
 * @param opt Options of state calculation: AVERAGE, WEIGHTED_AVERAGE, MAX_WEIGHT
 * @return Estimated state
 */
State estimateState(const Particles& particles, const StateCalculation opt);

} // namespace pfuclt::state

#endif //PFUCLT_STATE_HPP

#ifndef PFUCLT_PFUCLT_STATE_HPP
#define PFUCLT_PFUCLT_STATE_HPP

#include <vector>

#include "particle/subparticle.hpp"
#include "particle/particles.hpp"


namespace pfuclt::state {

struct States
{
  std::vector<pfuclt::particle::RobotSubparticle> robots;
  std::vector<pfuclt::particle::TargetSubparticle> targets;
};

class State
{
 public:
  // Conitainer where the state/pose of each robot and target is stored
  States states;

  // stores which targets are being seen by each robot
  std::vector<std::vector<bool>> targets_visibility;

  // stores the robot that just found a missing target
  std::vector<int> targets_found;

  State() = delete;
  State(const State &) = delete; // no copy
  State(State &&) = delete; // no move
  State& operator=(const State &) = delete; // no copy assign
  State& operator=(State &&) = delete; // no move assign

  /**
   * @brief Constructor of States
   * @param particles Pointer to instance of Particles class being used in the algorithm
   */
  State(pfuclt::particle::Particles* particles);

  /**
   * @brief Estimates the average of each subparticle set
   * @param particles Reference to an instance of the class Particles
   * @param state Reference to an instance of the struct State which will store
   * an estimate of the poses of robots and targets
   */
  void estimateAvg();

  /**
   * @brief Estimates the weighted average of each subparticle set
   * @param particles Reference to an instance of the class Particles
   * @param state Reference to an instance of the struct State which will store
   * an estimate of the poses of robots and targets
   */
  void estimateWeightedAvg();

  /**
   * @brief Retrieves the particle with maximum weigth
   * @param particles Reference to an instance of the class Particles
   * @param state Reference to an instance of the struct State which will store
   * the particle with maximum weight (excluding weight subparticle)
   */
  void estimateMaxWeight();

 private:
  // Pointer to particles
  const pfuclt::particle::Particles* particles_;
};

} // namespace pfuclt::state

#endif // PFUCLT_PFUCLT_STATE_HPP

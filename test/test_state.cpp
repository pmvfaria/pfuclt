#include <algorithm>
#include <numeric>

#include <gtest/gtest.h>

#include "pfuclt/state.hpp"
#include "particle/subparticle.hpp"
#include "particle/particles.hpp"

namespace pfuclt::testing {

using namespace ::pfuclt::state;

TEST(State, creation) {
  const int num_particles{300}, num_robots{4}, num_targets{3};

  pfuclt::particle::Particles particles(num_particles, num_robots, num_targets);

  State state(&particles);

  // Number of robots and targets and their initial values
  ASSERT_EQ(num_robots, state.states.robots.size());
  ASSERT_EQ(num_targets, state.states.targets.size());

  ASSERT_TRUE(std::all_of(state.states.robots.begin(), state.states.robots.end(),
                          [](pfuclt::particle::RobotSubparticle& robot_state) {
    for (int s = 0; s < robot_state.number_states; s++)
      if (robot_state[s] != 0.0) 
        return false;
    return true;
  }));
}

TEST(State, averages) {
  const int num_particles{300}, num_robots{4}, num_targets{3};

  pfuclt::particle::Particles particles(num_particles, num_robots, num_targets);
  // Initialize particles and compute the average
  const double d_left{5.0}, d_right{10.0};

  std::vector<std::array<double[2], particle::RobotSubparticle::number_states>>
      robots_vec((unsigned long) num_robots);
  std::vector<std::array<double[2], particle::TargetSubparticle::number_states>>
      targets_vec((unsigned long) num_targets);

  // First subscript is the robot index, second is the variable index, third is 0/1 for
  // min and max of uniform distribution
  robots_vec[0][1][0] = d_left;
  robots_vec[0][1][1] = d_right;

  targets_vec[0][2][0] = d_left;
  targets_vec[0][2][1] = d_right;
  particles.initialize(robots_vec, targets_vec);

  // Average
  double avg_robot_0_x = std::accumulate(particles.robots[0].begin(),
                                         particles.robots[0].end(), 0.0,
    [&](double sum, const pfuclt::particle::RobotSubparticle& rsp)
    {
      return sum + rsp.x;
    }) / num_particles;

  State state(&particles);
  state.estimateAvg();

  EXPECT_EQ(avg_robot_0_x, state.states.robots[0].x);

  // Weighted average
  double weighted_avg_robot_1_y = 0.0;
  int p = 0;
  for (const pfuclt::particle::RobotSubparticle& subparticle : particles.robots[1])
    weighted_avg_robot_1_y += subparticle.y * particles.weights[p++];

  state.estimateWeightedAvg();

  EXPECT_EQ(weighted_avg_robot_1_y, state.states.robots[1].y);

  // Maximum Weight
  const auto it = std::max_element(particles.weights.begin(), particles.weights.end());
  double max_weight_target_2_z = particles.targets[2][it - particles.weights.begin()].z;

  state.estimateMaxWeight();

  EXPECT_EQ(max_weight_target_2_z, state.states.targets[2].z);
}

} // namespace pfuclt:testing

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


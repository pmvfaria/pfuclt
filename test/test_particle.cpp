#include <algorithm>
#include <numeric>
#include <ostream>

#include <gtest/gtest.h>

#include "particle/subparticle.hpp"
#include "particle/particles.hpp"


namespace pfuclt::testing {

using namespace ::pfuclt::particle;

TEST(Particle, creation) { /* NOLINT */
  int num_particles{300}, num_robots{5}, num_targets{3};

  Particles particles(num_particles, num_robots, num_targets);

  // Number of sub-particle sets
  ASSERT_EQ(num_robots, particles.robots.size());
  ASSERT_EQ(num_targets, particles.targets.size());

  // Number of sub-particles in each sub-particle set
  ASSERT_TRUE(std::all_of(particles.robots.begin(), particles.robots.end(),
                          [num_particles](const RobotSubparticles& sp) {
                            return sp.size() == num_particles; } ));
  ASSERT_TRUE(std::all_of(particles.targets.begin(), particles.targets.end(),
                          [num_particles](const TargetSubparticles& sp) {
                            return sp.size() == num_particles; } ));
  ASSERT_EQ(num_particles, particles.weights.size());

  num_particles = 500;
  particles.resize(num_particles);
  // Number of sub-particles in each sub-particle set after resizing
  ASSERT_TRUE(std::all_of(particles.robots.begin(), particles.robots.end(),
                          [num_particles](const RobotSubparticles& sp) {
                            return sp.size() == num_particles; } ));
  ASSERT_TRUE(std::all_of(particles.targets.begin(), particles.targets.end(),
                          [num_particles](const TargetSubparticles& sp) {
                            return sp.size() == num_particles; } ));
  ASSERT_EQ(num_particles, particles.weights.size());

  std::ostringstream oss;
  EXPECT_NO_THROW(oss << particles << std::endl);
}

TEST(Particle, removeTargets) { /* NOLINT */
  const int num_particles{300}, num_robots{5}, num_targets{3};

  Particles particles(num_particles, num_robots, num_targets);

  // Remove two targets
  ASSERT_NO_THROW(particles.removeTarget(1).removeTarget(particles.targets.begin()));
  EXPECT_EQ(num_targets - 2, particles.targets.size());

  // There should not be a target with index 1 (0-indexed)
  EXPECT_THROW(particles.removeTarget(1), std::out_of_range);

  std::ostringstream oss;
  EXPECT_NO_THROW(oss << particles << std::endl);
}

TEST(Particle, sumOfWeights) { /* NOLINT */
  const int num_particles{1000};

  Particles p(num_particles, 1, 1);

  p.weights.assign(num_particles, 0.0);
  EXPECT_EQ(0.0, p.sumOfWeights());

  std::fill(p.weights.begin(), p.weights.begin() + 3, 5.0);
  EXPECT_EQ(15.0, p.sumOfWeights());

  std::fill(p.weights.begin(), p.weights.end(), 1.0);
  EXPECT_EQ(1000.0, p.sumOfWeights());
}

TEST(Particle, cornerCases) { /* NOLINT */
  Particles p_empty(0, 0, 0);
  EXPECT_EQ(0, p_empty.robots.size());
  EXPECT_EQ(0, p_empty.targets.size());
  EXPECT_EQ(0, p_empty.weights.size());
  EXPECT_THROW(p_empty.removeTarget(p_empty.targets.begin()), std::out_of_range);
  EXPECT_NO_THROW(p_empty.addTarget());
  EXPECT_EQ(1, p_empty.targets.size());

  std::ostringstream oss;
  EXPECT_NO_THROW(oss << p_empty << std::endl);

  // Division by zero in normalizing particle weights
  Particles p(10, 1, 2);
  p.weights.assign(p.num_particles, 0.0);
  EXPECT_THROW(p.normalizeWeights(), std::range_error);

  // Should keep their value of zero
  EXPECT_EQ(0.0, p.weights[0]);
}

TEST(Particle, normalizeAccuracy) { /* NOLINT */
  const double SMALL_WEIGHT{1e-15}; // expected precision
  const int num_particles = 1000;

  Particles p(num_particles, 3, 3);
  p.weights.assign(p.num_particles, SMALL_WEIGHT);

  // Value should not be 0, but near zero by SMALL_WEIGHT
  EXPECT_NE(0.0, p.weights[0]);
  EXPECT_NEAR(SMALL_WEIGHT, p.weights[0], 0.0);

  // Sum should be very near num_particles * SMALL_WEIGHT
  EXPECT_NEAR(num_particles * SMALL_WEIGHT,
              std::accumulate(p.weights.begin(), p.weights.end(), 0.0), SMALL_WEIGHT);

  // Normalize should not throw unless sum is exactly 0.0
  EXPECT_NO_THROW(p.normalizeWeights(std::nullopt));

  // Value after normalizing
  EXPECT_NEAR(SMALL_WEIGHT / (num_particles * SMALL_WEIGHT), p.weights[0], SMALL_WEIGHT);

  // Sum after normalizing
  EXPECT_NEAR(1.0, std::accumulate(p.weights.begin(), p.weights.end(), 0.0), SMALL_WEIGHT);
}

TEST(Particle, copyNormalizedWeights) { /* NOLINT */
  const int num_particles = 1000;
  Particles p(num_particles, 1, 1);

  p.weights.assign(num_particles, 1.0);

  const auto weights_copy = p.getNormalizedWeightsCopy();

  // Particles should have 1.0 weight each, but the copy should be normalized (total 1.0)
  EXPECT_EQ(num_particles * 1.0, std::accumulate(p.weights.begin(),p.weights.end(), 0.0));
  EXPECT_NEAR(1.0, std::accumulate(weights_copy.begin(), weights_copy.end(), 0.0), 1e-13);
}

TEST(Particle, initialization) { /* NOLINT */
  const int num_robots{2}, num_targets{3}, num_particles{100000};
  constexpr double TOLERANCE{1e-1};
  Particles p(num_particles, num_robots, num_targets);

  // Initialize with a specific distribution
  const double d_left{5.0}, d_right{10.0};

  std::vector<std::array<double[2], RobotSubparticle::number_states>>
      robots_vec((unsigned long) num_robots);
  std::vector<std::array<double[2], TargetSubparticle::number_states>>
      targets_vec((unsigned long) num_targets);

  // First subscript is the robot index, second is the variable index, third is 0/1 for
  // min and max of uniform distribution
  robots_vec[0][1][0] = d_left;
  robots_vec[0][1][1] = d_right;

  targets_vec[0][2][0] = d_left;
  targets_vec[0][2][1] = d_right;

  p.initialize(robots_vec, targets_vec);

  double avg_robot_0_y = std::accumulate(p.robots[0].begin(), p.robots[0].end(), 0.0,
                                            [&](double sum, const RobotSubparticle& rsp)
  {
    return sum + rsp.y;
  }) / num_particles;

  double avg_target_0_z = std::accumulate(p.targets[0].begin(), p.targets[0].end(), 0.0,
                                             [&](double sum, const TargetSubparticle& tsp)
  {
    return sum + tsp.z;
  }) / num_particles;

  EXPECT_NEAR(0.5 * (d_left + d_right), avg_robot_0_y, TOLERANCE);
  EXPECT_NEAR(0.5 * (d_left + d_right), avg_target_0_z, TOLERANCE);
}

} // namespace pfuclt:testing

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  return RUN_ALL_TESTS();
}

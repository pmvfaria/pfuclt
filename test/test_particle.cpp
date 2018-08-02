#include <gtest/gtest.h>
#include <algorithm>
#include <numeric>
#include <ostream>
#include <cmath>

#include "particle/subparticle.hpp"
#include "particle/particle.hpp"

int main(int argc, char **argv);

namespace pfuclt::testing{

using namespace ::pfuclt::particle;

TEST(Particle, creation){ /* NOLINT */
  const size_t num_particles = 300;
  const size_t num_robots = 5;
  const size_t num_targets = 3;

  Particles particles(num_particles, num_robots, num_targets);

  // Number of sub-particle sets
  EXPECT_EQ(num_robots, particles.robots.size());
  EXPECT_EQ(num_targets, particles.targets.size());

  // Number of particles in each sub-particle set
  EXPECT_TRUE(std::all_of(particles.robots.begin(), particles.robots.end(),
                          [](const RobotSubParticles& sp) { return sp.size() == num_particles; } ));
  EXPECT_TRUE(std::all_of(particles.targets.begin(), particles.targets.end(),
                          [](const TargetSubParticles& sp) { return sp.size() == num_particles; } ));
  EXPECT_EQ(num_particles, particles.weights.size());

  std::ostringstream oss;
  EXPECT_NO_THROW(oss << particles << std::endl);
}

TEST(Particle, removeTargets){ /* NOLINT */
  const size_t num_particles = 300;
  const size_t num_robots = 5;
  const size_t num_targets = 3;

  Particles particles(num_particles, num_robots, num_targets);

  // Remove two targets
  EXPECT_NO_THROW(particles.removeTarget(1).removeTarget(particles.targets.begin()));
  EXPECT_EQ(num_targets - 2, particles.targets.size());

  // There should not be a target with index 2 (0-indexed)
  EXPECT_THROW(particles.removeTarget(2), std::out_of_range);

  std::ostringstream oss;
  EXPECT_NO_THROW(oss << particles << std::endl);
}

TEST(Particle, cornerCases){ /* NOLINT */
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

TEST(Particle, normalizeAccuracy){ /* NOLINT */
  constexpr double SMALL_WEIGHT{1e-15}; // expected precision
  const size_t num_particles = 1000;

  Particles p(num_particles, 3, 3);
  p.weights.assign(p.num_particles, SMALL_WEIGHT);

  // Value should not be 0, but near zero by SMALL_WEIGHT
  EXPECT_NE(0.0, p.weights[0]);
  EXPECT_NEAR(SMALL_WEIGHT, p.weights[0],0.0);

  // Sum should be very near num_particles*SMALL_WEIGHT
  EXPECT_NEAR(num_particles*SMALL_WEIGHT, std::accumulate(p.weights.begin(), p.weights.end(), 0.0), SMALL_WEIGHT);

  // Normalize should not throw unless sum is exactly 0.0
  EXPECT_NO_THROW(p.normalizeWeights());

  // Value after normalizing
  EXPECT_NEAR(1.0/num_particles, p.weights[0], SMALL_WEIGHT);

  // Sum after normalizing
  EXPECT_NEAR(1.0, std::accumulate(p.weights.begin(), p.weights.end(), 0.0), SMALL_WEIGHT);
}

} // namespace pfuclt:testing

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
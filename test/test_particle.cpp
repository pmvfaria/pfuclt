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
  EXPECT_EQ(particles.robots.size(), num_robots);
  EXPECT_EQ(particles.targets.size(), num_targets);

  // Number of particles in each sub-particle set
  EXPECT_TRUE(std::all_of(particles.robots.begin(), particles.robots.end(),
                          [](const RobotSubParticles& sp) { return sp.size() == num_particles; } ));
  EXPECT_TRUE(std::all_of(particles.targets.begin(), particles.targets.end(),
                          [](const TargetSubParticles& sp) { return sp.size() == num_particles; } ));
  EXPECT_EQ(particles.weights.size(), num_particles);

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
  EXPECT_EQ(particles.targets.size(), num_targets - 2);

  // There should not be a target with index 2 (0-indexed)
  EXPECT_THROW(particles.removeTarget(2), std::out_of_range);

  std::ostringstream oss;
  EXPECT_NO_THROW(oss << particles << std::endl);
}

TEST(Particle, cornerCases){ /* NOLINT */
  Particles p_empty(0, 0, 0);
  EXPECT_EQ(p_empty.robots.size(), 0);
  EXPECT_EQ(p_empty.targets.size(), 0);
  EXPECT_EQ(p_empty.weights.size(), 0);
  EXPECT_THROW(p_empty.removeTarget(p_empty.targets.begin()), std::out_of_range);
  EXPECT_NO_THROW(p_empty.addTarget());
  EXPECT_EQ(p_empty.targets.size(), 1);

  std::ostringstream oss;
  EXPECT_NO_THROW(oss << p_empty << std::endl);

  // Division by zero in normalizing particle weights
  Particles p(10, 1, 2);
  p.weights.assign(p.num_particles, 0.0);
  EXPECT_THROW(p.normalizeWeights(), std::range_error);

  // Should keep their value of zero
  EXPECT_EQ(p.weights[0], 0.0);
}

} // namespace pfuclt:testing

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
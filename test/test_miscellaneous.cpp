#include <vector>
#include <algorithm>
#include <numeric>
#include <functional>
#include <cmath>

#include <gtest/gtest.h>

#include "pfuclt/state.hpp"
#include "thirdparty/stopwatch/StopWatch.h"


namespace pfuclt::testing {

using namespace ::pfuclt;

TEST(SortWeights, sortVector) {
  int num_particles = 8;
  std::vector<double> probabilities{0.1, 0.84, 0.74, 0.21, 0.65, 0.81, 0.01, 0.33};
  std::vector<double> probabilities_sorted(num_particles);
  
  EXPECT_FALSE(std::is_sorted(probabilities.begin(), probabilities.end(), std::greater<double>()));

  // Get sorted weights indices
  std::vector<int> idx(num_particles);
  std::iota(idx.begin(), idx.end(), 0);
  std::sort(idx.begin(), idx.end(),
    [&probabilities](auto i1, auto i2) { return probabilities[i1] > probabilities[i2]; });

  for (int i = 0; i < num_particles; ++i)
    probabilities_sorted[i] = probabilities[idx[i]];

  EXPECT_TRUE(std::is_sorted(probabilities_sorted.begin(), probabilities_sorted.end(), std::greater<double>()));
}

TEST(Watch, stopWatch) {
  StopWatch watch;
  sleep(1);
  std::cout << "Time passed: " << watch.ElapsedMs() << " ms" << std::endl;
}

TEST(Debug, avgError) {
  state::States error_, avg_error_;

  error_.robots.resize(4);
  avg_error_.robots.resize(4);

  for (particle::RobotSubparticle& robot : avg_error_.robots)
    for (int s = 0; s < robot.number_states; s++)
      robot[s] = 0.0;

  for (particle::RobotSubparticle& robot : error_.robots)
    for (int s = 0; s < robot.number_states; s++)
      robot[s] = 0.5;

  for (int r = 0; r < avg_error_.robots.size(); ++r)
    for (int s = 0; s < avg_error_.robots[r].number_states; ++s)
      avg_error_.robots[r][s] += std::abs(error_.robots[r][s]);

  EXPECT_EQ(avg_error_.robots[0].y, 0.5);

  for (particle::RobotSubparticle& robot : error_.robots)
    for (int s = 0; s < robot.number_states; s++)
      robot[s] = -0.1;

  for (int r = 0; r < avg_error_.robots.size(); ++r)
    for (int s = 0; s < avg_error_.robots[r].number_states; ++s)
      avg_error_.robots[r][s] = (avg_error_.robots[r][s] + std::abs(error_.robots[r][s])) / 2.0;

  EXPECT_EQ(avg_error_.robots[0].y, 0.3);
}

} // namespace pfuclt:testing

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

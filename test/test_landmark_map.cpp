#include <ostream>

#include <gtest/gtest.h>

#include "map/landmark_map.hpp"


namespace pfuclt::testing {

using namespace ::pfuclt::map;

TEST(LandmarkMap, mapAtOrigin) {  /* NOLINT */
  EXPECT_THROW(LandmarkMap(0, 0, -1, 5), std::invalid_argument);

  LandmarkMap map(0, 0, 3, 5);

  EXPECT_TRUE(map.landmarks.empty());

  EXPECT_FALSE(map.getAutoExpand());

  // Map has 1.5 meters in each side for x, so a landmark at x=2 should not be added
  EXPECT_FALSE(map.addLandmark(3, 1, 2, 1));

  map.setAutoExpand(true);
  EXPECT_TRUE(map.getAutoExpand());

  // Now it should be added by expanding the map
  EXPECT_TRUE(map.addLandmark(3, 1, 2, 1));

  EXPECT_EQ(3, map.landmarks[0].id);
  EXPECT_EQ(2.0, map.landmarks[0].x);
  EXPECT_EQ(1.0, map.landmarks[0].y);

  std::ostringstream oss;
  EXPECT_NO_THROW(oss << map << std::endl);
}

TEST(LandmarkMap, mapDecentered) {  /* NOLINT */
  EXPECT_THROW(LandmarkMap(2, -1, -1, 5), std::invalid_argument);

  LandmarkMap map(2, -1, 3, 5);

  EXPECT_TRUE(map.landmarks.empty());

  EXPECT_FALSE(map.getAutoExpand());

  // Map has 2.5 meters in each side for y, so a landmark at y=2 should not be added
  EXPECT_FALSE(map.addLandmark(3, 1, 2, 1));

  map.setAutoExpand(true);
  EXPECT_TRUE(map.getAutoExpand());

  // Now it should be added by expanding the map
  EXPECT_TRUE(map.addLandmark(3, 1, 2, 1));

  EXPECT_EQ(3, map.landmarks[0].id);
  EXPECT_EQ(2.0, map.landmarks[0].x);
  EXPECT_EQ(2.0, map.landmarks[0].y);

  std::ostringstream oss;
  EXPECT_NO_THROW(oss << map << std::endl);
}

} // namespace pfuclt:testing

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

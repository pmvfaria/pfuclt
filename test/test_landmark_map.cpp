#include <gtest/gtest.h>
#include <ostream>

#include "map/landmark_map.hpp"

int main(int argc, char **argv);

namespace pfuclt::testing{

using namespace ::pfuclt::map;

TEST(LandmarkMap, mapAtOrigin) { /* NOLINT */
  LandmarkMap map(0, 0, 3, 5);
  EXPECT_TRUE(map.landmarks.empty());

  map.setAutoExpand(false);
  EXPECT_FALSE(map.auto_expandable);

  // Map has 1.5 meters in each side for x, so a landmark at x=2 should not be added
  EXPECT_FALSE(map.addLandmark(3, 2, 1));

  map.setAutoExpand(true);
  EXPECT_TRUE(map.auto_expandable);

  // Now it should be added by expanding the map
  EXPECT_TRUE(map.addLandmark(3, 2, 1));
}

//TODO test map with offsets, test the actual sizes, etc

} // namespace pfuclt:testing

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
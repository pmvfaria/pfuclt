#include "sensor/odometry_data.hpp"
#include "map/map_ros.hpp"

int main(int argc, char **argv){

  using namespace pfuclt;

  ros::init(argc, argv, "pfuclt");
  ros::NodeHandle nh_world("/world");

  map::LandmarkMap map(0, 0, 12, 10);
  map.setAutoExpand(false);

  const auto num_landmarks = map::landmarksFromParameter(map, "landmarks", nh_world);
  ROS_INFO_STREAM( " Added " << num_landmarks << " landmarks");

  ROS_INFO_STREAM(map);

  return EXIT_SUCCESS;
}
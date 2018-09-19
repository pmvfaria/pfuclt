#include "sensor/odometry_data.hpp"
#include "map/map_ros.hpp"
#include "pfuclt/pfuclt.hpp"

int main(int argc, char **argv){

  ::ros::init(argc, argv, "pfuclt_node");
  ROS_INFO("PFUCLT node started");

  ::pfuclt::algorithm::PFUCLT pfuclt(1);

  pfuclt.run();

  return EXIT_SUCCESS;
}
#include "sensor/odometry_data.hpp"
#include "map/map_ros.hpp"
#include "pfuclt/pfuclt.hpp"

int main(int argc, char **argv){

  ros::init(argc, argv, "pfuclt_node");

//  ros::Time::waitForValid();

  ::pfuclt::algorithm::PFUCLT pfuclt(1);

  return EXIT_SUCCESS;
}
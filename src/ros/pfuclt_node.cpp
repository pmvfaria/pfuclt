#include <string>
#include <iostream>

#include "ros/ros.h"

#include "pfuclt/pfuclt.hpp"
#include "ros/globals.hpp"
#include "ros/debug.hpp"


namespace globals {

bool debug;
bool publish;
pfuclt::debug::Error* error;

} // namespace globals

int main(int argc, char* argv[])
{
  // Parse arguments
  if (argv[2] == std::string("true")) {
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
      ros::console::notifyLoggerLevelsChanged();

    globals::debug = true;
  }
  else
    globals::debug = false;
  
  if (argv[4] == std::string("true"))
    globals::publish = true;
  else
    globals::publish = false;

  ROS_INFO_STREAM("DEBUG set to " << std::boolalpha << globals::debug << " and PUBLISH set to " << std::boolalpha << globals::publish);

  ros::init(argc, argv, "pfuclt_node");
  ROS_INFO_STREAM("PFUCLT node started");

  pfuclt::algorithm::PFUCLT pfuclt(1);

  if (globals::debug)
    globals::error = new pfuclt::debug::Error(pfuclt);

  pfuclt.run();
  
  if (globals::debug)
    delete globals::error;

  return EXIT_SUCCESS;
}

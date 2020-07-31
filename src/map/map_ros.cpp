#include "map/map_ros.hpp"

#include "xmlrpcpp/XmlRpcException.h"


namespace pfuclt::map {

void wallsFromParameter(std::vector<double>& walls,
                        const std::string& param,
                        const ros::NodeHandle& nh)
{
  using namespace XmlRpc;

  XmlRpcValue walls_raw;
  try {
    if (!nh.getParam(param, walls_raw))
      ROS_FATAL_STREAM("Walls parameter not found: " << nh.resolveName(param));

    ROS_ASSERT_MSG(walls_raw.getType() == XmlRpcValue::TypeStruct,
                   "Expected walls as a struct");

    ROS_ASSERT_MSG(static_cast<size_t>(walls_raw.size()) == walls.size(),
                   "Struct walls should have size %ld", walls.size());

    std::string walls_string[walls.size()] = { "ceiling", "down", "floor", "left", "right", "up" };
    int i = 0;
    for (auto it = walls_raw.begin(); it != walls_raw.end(); ++it) {
      ROS_ASSERT_MSG(it->first == walls_string[i],
                     "First member of walls parameter (%s) doesn't correspond to first key in string to compare (%s)",
                     it->first.c_str(), walls_string[i].c_str());
      ROS_ASSERT_MSG(it->second.getType() == XmlRpcValue::TypeDouble || it->second.getType() == XmlRpcValue::TypeInt,
                     "Expected value of %s as a double or integer",
                     it->first.c_str());

      walls[i] = static_cast<double>(it->second);
      i++;
    }

    // Walls ordered alphabetically: [ceiling, down, floor, left, right, up]
    ROS_ASSERT_MSG(walls[1] < walls[5],
                   "Value of %s must be smaller than value of %s",
                   walls_string[1].c_str(), walls_string[5].c_str());
    ROS_ASSERT_MSG(walls[3] < walls[4],
                   "Value of %s must be smaller than value of %s",
                   walls_string[3].c_str(), walls_string[4].c_str());
  }
  catch (const XmlRpcException& e) {
    ROS_FATAL_STREAM(e.getMessage());
    throw;
  }
}


int landmarksFromParameter(LandmarkMap& map, const std::string& param, const ros::NodeHandle& nh)
{
  using namespace XmlRpc;

  XmlRpcValue landmarks_raw;
  try {
    if (!nh.getParam(param, landmarks_raw))
      ROS_FATAL_STREAM("Landmarks parameter not found: " << nh.resolveName(param));
  
    ROS_ASSERT_MSG(landmarks_raw.getType() == XmlRpcValue::TypeArray,
                   "Expected landmarks as a list");

    for (int lm_id = 0; lm_id < landmarks_raw.size(); ++lm_id) {
      ROS_ASSERT_MSG(landmarks_raw[lm_id].getType() == XmlRpcValue::TypeArray &&
                     landmarks_raw[lm_id].size() == 3,
                     "Expected a list with two values for each landmark");

      const double lm_x = static_cast<double>(landmarks_raw[lm_id][0]);
      const double lm_y = static_cast<double>(landmarks_raw[lm_id][1]);
      const double lm_height = static_cast<double>(landmarks_raw[lm_id][2]);

      if (!map.addLandmark(lm_id, lm_height, lm_x, lm_y))
        ROS_ERROR_STREAM("Could not add landmark at " <<
                         "{x= " << lm_x << ", y=" << lm_y << "} ... Map too small?");
    }
  }
  catch (const XmlRpcException& e) {
    ROS_FATAL_STREAM(e.getMessage());
    throw;
  }

  return map.landmarks.size();
}

} // namespace pfuclt::map

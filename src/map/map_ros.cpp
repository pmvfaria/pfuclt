//
// Created by glawless on 8/8/18.
//

#include <xmlrpcpp/XmlRpcException.h>
#include "map_ros.hpp"

namespace pfuclt::map {

std::size_t landmarksFromParameter(LandmarkMap& map, const std::string &param, const ros::NodeHandle &nh) {
  XmlRpc::XmlRpcValue landmarks_raw;
  try{
    nh.getParam(param, landmarks_raw);
    ROS_ASSERT_MSG(landmarks_raw.getType() == XmlRpc::XmlRpcValue::TypeArray, "Expected landmarks as a list");

    for(int lm_id=0; lm_id<landmarks_raw.size(); ++lm_id) { /* NOLINT */
      ROS_ASSERT_MSG(landmarks_raw[lm_id].getType() == XmlRpc::XmlRpcValue::TypeArray, "Expected each landmark as a list");

      const auto lm_x = static_cast<double>(landmarks_raw[lm_id][0]);
      const auto lm_y = static_cast<double>(landmarks_raw[lm_id][1]);

      if(!map.addLandmark(lm_id, lm_x, lm_y))
        ROS_ERROR_STREAM("Could not add landmark at " << "{x= " << lm_x << ", y=" << lm_y << "} ... Map too small?");
    }
  }
  catch (const XmlRpc::XmlRpcException& e){
    ROS_FATAL_STREAM(e.getMessage());
    throw;
  }

  return map.landmarks.size();
}

} // namespace pfuclt::map
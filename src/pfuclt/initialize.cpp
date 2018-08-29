//
// Created by glawless on 8/23/18.
//

#include "pfuclt.hpp"
#include <xmlrpcpp/XmlRpcException.h>

namespace pfuclt::algorithm {

const std::size_t PFUCLT::getLandmarkMap() {

  ros::NodeHandle wnh{"/world"};

  map_ = std::make_unique<::pfuclt::map::LandmarkMap>(0, 0, 12, 10);
  map_->setAutoExpand(false);

  return ::pfuclt::map::landmarksFromParameter(*map_, "landmarks", wnh);
}

const bool PFUCLT::initializeParticles() {

  using namespace ::pfuclt::particle;
  using namespace ::XmlRpc;

  const std::string init_robots_param{"init_robot_particles"};
  const std::string init_targets_param{"init_target_particles"};

  if (pnh_.hasParam(init_robots_param) && pnh_.hasParam(init_targets_param)) {

    try {
      XmlRpcValue init_robots, init_targets;

      pnh_.getParam(init_robots_param, init_robots);
      ROS_ASSERT_MSG(init_robots.getType() == XmlRpcValue::TypeArray,
                     "Expected %s as a list",
                     init_robots_param.c_str());

      // Vector to hold and later send distribution values
      std::vector<std::array<double[2], RobotSubParticle::number_states>>
          robots_vec((unsigned long) init_robots.size());

      for (int robot = 0; robot < init_robots.size(); ++robot) {
        ROS_ASSERT_MSG(init_robots[robot].getType() == XmlRpcValue::TypeArray, "Expected a list for each robot");

        for (int var = 0; var < init_robots[robot].size(); ++var) {
          ROS_ASSERT_MSG(
              init_robots[robot][var].getType() == XmlRpcValue::TypeArray,
              "Expected a list with two values for each variable");

          robots_vec[robot][var][0] = static_cast<double>(init_robots[robot][var][0]);
          robots_vec[robot][var][1] = static_cast<double>(init_robots[robot][var][1]);
        }
      }

      pnh_.getParam(init_targets_param, init_targets);
      ROS_ASSERT_MSG(init_targets.getType() == XmlRpcValue::TypeArray,
                     "Expected %s as a list",
                     init_targets_param.c_str());

      // Vector to hold and later send distribution values
      std::vector<std::array<double[2], TargetSubParticle::number_states>>
          targets_vec((unsigned long) init_targets.size());

      for (int target = 0; target < init_targets.size(); ++target) {
        ROS_ASSERT_MSG(init_targets[target].getType() == XmlRpcValue::TypeArray, "Expected a list for each target");

        for (int var = 0; var < init_targets[target].size(); ++var) {
          ROS_ASSERT_MSG(
              init_targets[target][var].getType() == XmlRpcValue::TypeArray && init_targets[target][var].size() == 2,
              "Expected a list with two values for each variable");

          targets_vec[target][var][0] = static_cast<double>(init_targets[target][var][0]);
          targets_vec[target][var][1] = static_cast<double>(init_targets[target][var][1]);
        }
      }
      particles_->initialize(robots_vec, targets_vec);
      return true;
    }
    catch (const XmlRpcException &e) {
      ROS_FATAL_STREAM(e.getMessage());
      throw;
    }

  } else {
    particles_->initialize();
    return false;
  }
}

} // namespace pfuclt::algorithm
#include <memory>
#include <string>
#include <array>
#include <vector>
#include <cmath>

#include "ros/ros.h"
#include "xmlrpcpp/XmlRpcException.h"

#include "pfuclt.hpp"
#include "particle/subparticle.hpp"


namespace pfuclt {

using namespace pfuclt;

namespace algorithm {

int PFUCLT::getLandmarkMap()
{
  ros::NodeHandle wnh{"/world"};

  // Walls ordered alphabetically: [ceiling, down, floor, left, right, up]
  std::vector<double> walls(6);
  map::wallsFromParameter(walls, "walls", wnh);

  double size_x = walls[4] - walls[3];
  double size_y = walls[5] - walls[1];
  double center_x = walls[3] + size_x / 2.0;
  double center_y = walls[1] + size_y / 2.0;

  map_ = std::make_unique<map::LandmarkMap>(center_x, center_y, size_x, size_y);

  return map::landmarksFromParameter(*map_, "landmarks", wnh);
}

void PFUCLT::getCovariances()
{
  using namespace XmlRpc;

  const std::string cov_landmarks {"landmark_covariances"};
  const std::string cov_targets {"target_covariances"};
 
  XmlRpcValue cov_landmarks_raw, cov_targets_raw;

  try {
    if (!pnh_.getParam(cov_landmarks, cov_landmarks_raw))
      ROS_FATAL_STREAM("Landmark Covariances parameter not found: " << pnh_.resolveName(cov_landmarks));

    ROS_ASSERT_MSG(cov_landmarks_raw.getType() == XmlRpcValue::TypeArray &&
                  static_cast<size_t>(cov_landmarks_raw.size()) == sensor::measurement::K_landmarks.size(),
                  "Expected %s as a list (type %d) with %d values",
                  pnh_.resolveName(cov_landmarks).c_str(),
                  cov_landmarks_raw.getType(),
                  static_cast<int>(sensor::measurement::K_landmarks.size()));

    for (int i = 0; i < cov_landmarks_raw.size(); ++i) {
      sensor::measurement::K_landmarks[i] = static_cast<double>(cov_landmarks_raw[i]);
      ROS_ASSERT_MSG(sensor::measurement::K_landmarks[i] > 0, "Parameters of %s must be larger than 0",
                                                              pnh_.resolveName(cov_landmarks).c_str());
    }

    if (!pnh_.getParam(cov_targets, cov_targets_raw))
      ROS_FATAL_STREAM("Target Covariances parameter not found: " << pnh_.resolveName(cov_targets));

    ROS_ASSERT_MSG(cov_targets_raw.getType() == XmlRpcValue::TypeArray &&
                  static_cast<size_t>(cov_targets_raw.size()) == sensor::measurement::K_targets.size(),
                  "Expected %s as a list (type %d) with %d values",
                  pnh_.resolveName(cov_targets).c_str(),
                  cov_targets_raw.getType(),
                  static_cast<int>(sensor::measurement::K_targets.size()));

    for (int i = 0; i < cov_targets_raw.size(); ++i) {
      sensor::measurement::K_targets[i] = static_cast<double>(cov_targets_raw[i]);
      ROS_ASSERT_MSG(sensor::measurement::K_targets[i] > 0, "Parameters of %s must be larger than 0",
                                                            pnh_.resolveName(cov_targets).c_str());
    }
  }
  catch (const XmlRpcException& e) {
    ROS_FATAL_STREAM(e.getMessage());
    throw;
  }
}


bool PFUCLT::initializeParticles()
{
  using namespace pfuclt::particle;
  using namespace XmlRpc;

  const std::string init_robots{"init_robot_particles"};
  const std::string init_targets{"init_target_particles"};

  XmlRpcValue init_robots_raw, init_targets_raw;

  try {
    if (!pnh_.getParam(init_robots, init_robots_raw))
      ROS_FATAL_STREAM("Initial Robot Particles parameter not found: " << pnh_.resolveName(init_robots));

    ROS_ASSERT_MSG(init_robots_raw.getType() == XmlRpcValue::TypeArray,
                    "Expected %s as a list", init_robots.c_str());

    // Vector to hold and later send distribution values
    std::vector<std::array<double[2], RobotSubparticle::number_states>>
        robots_vec(static_cast<size_t>(init_robots_raw.size()));

    for (int robot = 0; robot < init_robots_raw.size(); ++robot) {
      ROS_ASSERT_MSG(init_robots_raw[robot].getType() == XmlRpcValue::TypeArray,
                      "Expected a list for each robot");

      for (int state = 0; state < init_robots_raw[robot].size(); ++state) {
        ROS_ASSERT_MSG(init_robots_raw[robot][state].getType() == XmlRpcValue::TypeArray &&
                        init_robots_raw[robot][state].size() == 2,
                        "Expected a list with two values for each state");

        robots_vec[robot][state][0] = static_cast<double>(init_robots_raw[robot][state][0]);
        robots_vec[robot][state][1] = static_cast<double>(init_robots_raw[robot][state][1]);
      }
    }

    if (pnh_.hasParam(init_targets)) {
      pnh_.getParam(init_targets, init_targets_raw);
      ROS_ASSERT_MSG(init_targets_raw.getType() == XmlRpcValue::TypeArray,
                     "Expected %s as a list", init_targets.c_str());

      // Vector to hold and later send distribution values
      std::vector<std::array<double[2], TargetSubparticle::number_states>>
          targets_vec(static_cast<size_t>(init_targets_raw.size()));

      for (int target = 0; target < init_targets_raw.size(); ++target) {
        ROS_ASSERT_MSG(init_targets_raw[target].getType() == XmlRpcValue::TypeArray,
                       "Expected a list for each target");

        for (int state = 0; state < init_targets_raw[target].size(); ++state) {
          ROS_ASSERT_MSG(init_targets_raw[target][state].getType() == XmlRpcValue::TypeArray &&
                          init_targets_raw[target][state].size() == 2,
                          "Expected a list with two values for each state");

          targets_vec[target][state][0] = static_cast<double>(init_targets_raw[target][state][0]);
          targets_vec[target][state][1] = static_cast<double>(init_targets_raw[target][state][1]);
        }
      }
      particles_->initialize(robots_vec, targets_vec);
      return true;
    }
    else {
      std::array<double[2], TargetSubparticle::number_states> target_dist;

      for (size_t s = 0; s < TargetSubparticle::number_states; ++s) {
        target_dist[s][0] = -10;
        target_dist[s][1] = 10;
      }

      std::vector<std::array<double[2], TargetSubparticle::number_states>>
          targets_vec(num_targets, target_dist);

      particles_->initialize(robots_vec, targets_vec);
      return false;
    }
  }
  catch (const XmlRpcException &e) {
    ROS_FATAL_STREAM(e.getMessage());
    throw;
  }
}

} // namespace algorithm

namespace robot {

void Robot::initialize()
{
  using namespace XmlRpc;

  ros::NodeHandle anh{"/alphas"};

  const std::string& alphas = name;

  XmlRpcValue alphas_raw;

  try {
    if (!anh.getParam(alphas, alphas_raw))
      ROS_FATAL_STREAM("Robot Alphas parameter not found: " << anh.resolveName(alphas));

    ROS_ASSERT_MSG(alphas_raw.getType() == XmlRpcValue::TypeArray,
                  "Expected %s as a list", anh.resolveName(alphas).c_str());

    alphas_.reserve(alphas_raw.size());
    for (int i = 0; i < alphas_raw.size(); ++i)
      alphas_.emplace_back(static_cast<double>(alphas_raw[i]));
  }
  catch (const XmlRpcException &e) {
    ROS_FATAL_STREAM(e.getMessage());
    throw;
  }
}

} // namespace robot

namespace target {

void Target::initialize()
{
  ros::NodeHandle pgnh{"~"};

  const std::string stddev_param{"predict_model_stddev"};

  if (!pgnh.getParam(stddev_param, param_stddev_))
    ROS_FATAL_STREAM("Target Std Deviation parameter not found: " << pgnh.resolveName(stddev_param));

  motion_mean_ = 0.0;
  motion_stddev_ = param_stddev_;

  last_motion = ros::Time::now();
}

} // namespace target

} // namespace pfuclt
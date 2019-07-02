//
// Created by glawless on 9/4/18.
//

#ifndef PFUCLT_ROBOT_HPP
#define PFUCLT_ROBOT_HPP

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <random>
#include <queue>
#include <mutex>
#include <cmath>
#include <Eigen/Dense>

#include <clt_msgs/CustomOdometry.h>

#include "../particle/particles.hpp"
#include "../map/landmark_map.hpp"
#include "../sensor/odometry_data.hpp"
#include "../sensor/target_data.hpp"
#include "../sensor/landmark_data.hpp"

namespace pfuclt::robot {

using namespace ::pfuclt::sensor;

using generator_type = std::mt19937;


/**
 * @brief The Robot class - Has the common variables and methods of all robots,
 * and is the base class of any specialized robots who may derive from it.
 * All robots should be instances of this class
 */
class Robot {

 public:
  // id of this robot - they should start at 0
  const uint idx;

  // name of this robot - should end with a number
  const std::string name;

 private:
  std::random_device rd_{};
  generator_type generator_;

  static constexpr auto name_prefix_ = "robot";

  std::array<double, 4> alphas_;

  // scoped namespace
  ::ros::NodeHandle nh_;

  // subscriber and queue to take odometry messages
  ros::Subscriber odometry_sub_;
  std::queue<odometry::OdometryMeasurement> odometry_cache_;
  std::mutex odometry_mutex_;

  // subscriber and queue to take target messages
  ros::Subscriber target_sub_;
  std::queue<target_data::TargetMeasurement> target_cache_;

  // subscriber and queue to take landmark messages
  ros::Subscriber landmark_sub_;
  std::queue<landmark::LandmarkMeasurements> landmark_cache_;

 public:
  // pointer to this robot's sub-particles
  particle::RobotSubParticles *subparticles;
  std::mutex subparticles_mutex_;

  // pointer to landmark map
  map::LandmarkMap *map;


 private:
  void getAlphas();
  
  /**
   * @brief Event-driven function that should be called when
   * new odometry data is received
   */
  void odometryCallback(const clt_msgs::CustomOdometryConstPtr&);
  /**
   * @brief Use robot motion model to estimate new particle's state
   * for this robot.
   * @details Odometry retrieved in method odometryCallback is used
   * to calculate the new particle state.
   */
  void processOldestOdometry();

  /**
   * @brief Event-driven function that should be called when
   * new target data is received
   */
  void targetCallback(const clt_msgs::MeasurementStampedConstPtr&);
  void processTargetMeasurement();

  /**
   * @brief Event-driven function which should be called when
   * new landmark data is received
   */
  void landmarkCallback(const clt_msgs::MeasurementArrayConstPtr&);
  void processLandmarkMeasurement();


 public:
  Robot() = delete;
  Robot(const Robot &) = delete; // no copy
  Robot(Robot &&) = delete; // no move
  Robot& operator=(const Robot &) = delete; // no copy assign
  Robot& operator=(Robot &&) = delete; // no move assign

  /**
   * Constructor
   * @param idx the id of this robot (usually should start at 0)
   * @param subparticles pointer to the subparticles of these robot in a set of particles
   */
  Robot(uint id, particle::RobotSubParticles* p_subparticles, map::LandmarkMap* map);

  /**
   * @brief Process all cached odometry messages, sampling the motion model for each particle
   */
  void predict();

  void update();
  
  //TODO: Robot(std::string name);
};

} // namespace pfuclt::robot

#endif //PFUCLT_ROBOT_HPP

//
// Created by glawless on 9/4/18.
//

#ifndef PFUCLT_ROBOT_HPP
#define PFUCLT_ROBOT_HPP

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <random>
#include <queue>

#include <clt_msgs/CustomOdometry.h>

#include "../particle/particles.hpp"
#include "../sensor/odometry_data.hpp"

namespace pfuclt::robot {

using namespace ::pfuclt::sensor;

class Robot {

 public:
  // id of this robot - they should start at 0
  const uint idx;

  // name of this robot - should end with a number
  const std::string name;

 private:
  long seed_{std::chrono::system_clock::now().time_since_epoch().count()};
  std::mt19937 generator_{seed_};

  static constexpr auto name_prefix_ = "robot";

  std::array<double, 4> alphas_;

  // scoped namespace
  ::ros::NodeHandle nh_;

  // subscriber and queue to take odometry messages
  ros::Subscriber odometry_sub_;
  std::queue<odometry::OdometryMeasurement> odometry_cache_;

 public:
  // pointer to this robot's sub-particles
  particle::RobotSubParticles *subparticles;

 private:
  void getAlphas();
  void odometryCallback(const clt_msgs::CustomOdometryConstPtr&);
  void processOdometryUntil(const ros::Time& t);
  void processOdometryMeasurement(const clt_msgs::CustomOdometryConstPtr &msg) const;


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
   * @param odometry_cb_queue pointer to a callback queue to associate with the odometry subscriber
   */
  Robot(uint id, particle::RobotSubParticles* p_subparticles, ros::CallbackQueue* odometry_cb_queue);

  //TODO Robot(std::string name);
};

} // namespace pfuclt::robot

#endif //PFUCLT_ROBOT_HPP

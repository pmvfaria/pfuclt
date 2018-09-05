//
// Created by glawless on 9/4/18.
//

#ifndef PFUCLT_ROBOT_HPP
#define PFUCLT_ROBOT_HPP

#include <ros/ros.h>
#include <queue>

#include <clt_msgs/CustomOdometry.h>

#include "../particle/particles.hpp"
#include "../sensor/odometry_data.hpp"

namespace pfuclt::robot {

class Robot {

 public:
  // id of this robot - they should start at 1
  const uint idx;

  // name of this robot - should end with a number
  const std::string name;

 private:
  static constexpr auto name_prefix_ = "robot";

  // scoped namespace
  ::ros::NodeHandle nh_;

  // subscriber and queue to take odometry messages
  ros::Subscriber odometry_sub_;
  std::queue<sensor::odometry::OdometryMeasurement> odometry_queue_;

 public:
  // pointer to this robot's sub-particles
  particle::RobotSubParticles *subparticles;

 private:
  void odometryCallback(const clt_msgs::CustomOdometryConstPtr&);

 public:

  Robot() = delete;

  /**
   * Constructor
   * @param idx the id of this robot (usually should start at 1)
   * @param subparticles pointer to the subparticles of these robot in a set of particles
   */
  Robot(const uint idx, particle::RobotSubParticles* subparticles);

  //TODO Robot(std::string name);
};

} // namespace pfuclt::robot

#endif //PFUCLT_ROBOT_HPP

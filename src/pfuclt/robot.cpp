//
// Created by glawless on 9/4/18.
//

#include "robot.hpp"
#include <random>
#include <angles/angles.h>

namespace pfuclt::robot {

Robot::Robot(const uint id, particle::RobotSubParticles* p_subparticles)
  : idx(id), name(Robot::name_prefix_ + std::to_string(idx+1)),
  generator_(rd_()), nh_("/robots/"+name), subparticles(p_subparticles) {

  getAlphas();

  odometry_sub_ = nh_.subscribe("/odometry", 100, &Robot::odometryCallback, this);
  target_sub_ = nh_.subscribe("/target", 100, &Robot::targetCallback, this);
  landmark_sub_ = nh_.subscribe("/landmark", 100, &Robot::landmarkCallback, this);
}

void Robot::odometryCallback(const clt_msgs::CustomOdometryConstPtr &msg) {
  std::lock_guard<std::mutex> lock(odometry_mutex_);
  odometry_cache_.emplace(odometry::fromRosCustomMsg(msg));
}

void Robot::processOldestOdometry() {

  // Retrieve oldest element of queue
  std::unique_lock<std::mutex> odometry_lock(odometry_mutex_);
  auto odom = odometry_cache_.front();
  odometry_cache_.pop();
  odometry_lock.unlock();

  // Robot motion model (from Probabilistic Robotics book)
  std::normal_distribution<double>
      rot1_rand(
          odom.initial_rotation,
          alphas_[0] * fabs(odom.initial_rotation) + alphas_[1] * odom.translation),
      trans_rand(
          odom.translation,
          alphas_[2] * odom.translation + alphas_[3] * fabs(odom.initial_rotation + odom.final_rotation)
          ),
      rot2_rand(
          odom.final_rotation,
          alphas_[0] * fabs(odom.final_rotation) + alphas_[1] * odom.translation);

  {
    std::lock_guard<std::mutex> subparticles_lock(subparticles_mutex_);
    for(auto& particle : *subparticles) {

      // Rotate to final position
      particle.theta += rot1_rand(generator_);

      // Sample and translate
      const double sample_translation = trans_rand(generator_);
      particle.x += sample_translation * cos(particle.theta);
      particle.y += sample_translation * sin(particle.theta);

      // Rotate to final and normalize
      particle.theta = angles::normalize_angle(particle.theta + rot2_rand(generator_));
    }
  }
}

void Robot::predict() {
  while (!odometry_cache_.empty())
  {
    processOldestOdometry();
  }
}

void Robot::targetCallback(const clt_msgs::MeasurementStampedConstPtr &msg) {
  //target_cache_.emplace(target_data::fromRosMsg(msg));
}

void Robot::processTargetMeasurement() {
  
}

void Robot::landmarkCallback(const clt_msgs::MeasurementArrayConstPtr &msg) {
  landmark_cache_.emplace(landmark::fromRosMsg(msg));
}

void Robot::processLandmarkMeasurement() {

  // Retrieve oldest element of queue
  auto lk = landmark_cache_.front();
  landmark_cache_.pop();  // Observation in robot frame

  // TODO: Convert landmark position in global frame (x,y) to robot frame (range,bearing)

  for (auto& landmark: lk.measurements) {

    
  }

}

} // namespace pfuclt::robot
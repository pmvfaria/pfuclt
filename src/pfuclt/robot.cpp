//
// Created by glawless on 9/4/18.
//

#include "robot.hpp"
#include <random>
#include <angles/angles.h>

namespace pfuclt::robot {

Robot::Robot(const uint id, particle::RobotSubParticles* p_subparticles, ros::CallbackQueue* odometry_cb_queue)
  : idx(id), name(Robot::name_prefix_ + std::to_string(idx+1)),
  nh_("/robots/"+name), subparticles(p_subparticles) {

  getAlphas();

  nh_.setCallbackQueue(odometry_cb_queue);
  odometry_sub_ = nh_.subscribe( "odometry", 100,  &Robot::odometryCallback, this);

}

void Robot::odometryCallback(const clt_msgs::CustomOdometryConstPtr &msg) {
  odometry_cache_.emplace(odometry::fromRosCustomMsg(msg));
}

void Robot::processOdometryUntil(const ros::Time& t) {


}

void Robot::processOdometryMeasurement(const clt_msgs::CustomOdometryConstPtr &msg) const {

  // Robot motion model (from Probabilistic Robotics book)
  std::normal_distribution<double>
      rot1_rand(
          msg->rot1,
          alphas_[0] * fabs(msg->rot1) + alphas_[1] * msg->translation),
      trans_rand(
          msg->translation,
          alphas_[2] * msg->translation + alphas_[3] * fabs(msg->rot1 + msg->rot2)
          ),
      rot2_rand(
          msg->rot2,
          alphas_[0] * fabs(msg->rot2) + alphas_[1] * msg->translation);

  for(auto& particle: *subparticles) {

    // Rotate to final position
    particle.theta() += rot1_rand(generator_);

    // Sample and translate
    const double sample_translation = trans_rand(generator_);
    particle.x() += sample_translation * cos(particle.theta());
    particle.y() += sample_translation * sin(particle.theta());

    // Rotate to final and normalize
    particle.theta() = angles::normalize_angle(particle.theta() + rot2_rand(generator_));
  }
}

} // namespace pfuclt::robot
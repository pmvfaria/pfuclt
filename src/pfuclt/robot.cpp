//
// Created by glawless on 9/4/18.
//

#include "robot.hpp"

namespace pfuclt::robot {

Robot::Robot(const uint id, particle::RobotSubParticles *subparticles)
    : idx(id), name(Robot::name_prefix_ + std::to_string(idx)), nh_("/robots/" + name), subparticles(subparticles) {

  odometry_sub_ = nh_.subscribe( "odometry", 100, &Robot::odometryCallback, this);

}

void Robot::odometryCallback(const clt_msgs::CustomOdometryConstPtr &msg) {
  odometry_queue_.emplace(
        sensor::odometry::fromRosCustomMsg(msg)
        );
}

} // namespace pfuclt::robot
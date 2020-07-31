#include "pfuclt_publisher.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PointStamped.h"
#include "sensor_msgs/PointCloud.h"

#include "pfuclt/pfuclt.hpp"
#include "pfuclt/state.hpp"
#include "particle/particles.hpp"


namespace pfuclt::publisher {

using namespace pfuclt;

PfucltPublisher::PfucltPublisher(algorithm::PFUCLT* pfuclt)  
  : pfuclt_(pfuclt),
    target_state_publisher_(pfuclt_->num_targets),
    target_particles_publisher_(pfuclt_->num_targets),
    robot_state_publisher_(pfuclt_->num_robots),
    robot_particles_publisher_(pfuclt_->num_robots)
{
  // Robots
  for (const auto& robot : pfuclt_->robots_) {
    robot_state_publisher_[robot->idx] =
        nh_.advertise<geometry_msgs::PoseStamped>(
            "/robots/" + robot->name + "/estimated_pose", 10);

    robot_particles_publisher_[robot->idx] =
        nh_.advertise<geometry_msgs::PoseArray>(
            "/robots/" + robot->name + "/particles", 10);

    // Build estimate message
    estimate_msg_.robots_estimate.emplace_back(geometry_msgs::PoseStamped());
  }

  // Targets
  for (const auto& target : pfuclt_->targets_) {
    target_state_publisher_[target->idx] =
        nh_.advertise<geometry_msgs::PointStamped>(
            "/targets/" + target->name + "/estimated_point", 10);

    target_particles_publisher_[target->idx] =
        nh_.advertise<sensor_msgs::PointCloud>(
            "/targets/" + target->name + "/particles", 10);

    // Build estimate message
    estimate_msg_.targets_estimate.emplace_back(geometry_msgs::PointStamped());
    estimate_msg_.targets_visibility.emplace_back(false);
  }

  estimate_publisher_ = nh_.advertise<clt_msgs::Estimate>("/estimate", 10);
}

void PfucltPublisher::publishRobotParticles()
{
  // Publish a series of PoseArray messages for each robot
  for (const auto& robot : pfuclt_->robots_) {
    geometry_msgs::PoseArray msg_robot_particles;

    msg_robot_particles.header.frame_id = "world";

    particle::RobotSubparticles& robot_subparticles =
        pfuclt_->particles_->robots[robot->idx];

    for (const auto& subparticle : robot_subparticles) {
      // Rotation around z axis with a magnitude of yaw rad
      tf2::Quaternion quaternion(tf2::Vector3(0, 0, 1), subparticle.yaw);
      // Rotation from quaternion & translation from vector
      tf2::Transform transform(
          quaternion, tf2::Vector3(subparticle.x, subparticle.y, subparticle.z));
      geometry_msgs::Pose pose;
      tf2::toMsg(transform, pose);
      msg_robot_particles.poses.insert(msg_robot_particles.poses.begin(), pose);
    }

    robot_particles_publisher_[robot->idx].publish(msg_robot_particles);
  }
}

void PfucltPublisher::publishTargetParticles()
{
  // Publish a series of PointCloud messages for each target
  for (const auto& target : pfuclt_->targets_) {
    sensor_msgs::PointCloud msg_target_particles;
    msg_target_particles.header.stamp = target->last_motion;
    msg_target_particles.header.frame_id = "world";

    particle::TargetSubparticles& target_subparticles =
        pfuclt_->particles_->targets[target->idx];

    for (const auto& subparticle : target_subparticles) {
      geometry_msgs::Point32 point;
      point.x = subparticle.x;
      point.y = subparticle.y;
      point.z = subparticle.z;

      msg_target_particles.points.insert(msg_target_particles.points.begin(), point);
    }

    target_particles_publisher_[target->idx].publish(msg_target_particles);
  }
}

void PfucltPublisher::publishRobotState()
{
  for (const auto& robot : pfuclt_->robots_) {
    geometry_msgs::PoseStamped& pose_stamped = estimate_msg_.robots_estimate[robot->idx];

    pose_stamped.header.frame_id = "world";

    const particle::RobotSubparticle& state = pfuclt_->state_->states.robots[robot->idx];

    // Rotation around z axis with a magnitude of yaw rad
    tf2::Quaternion quaternion(tf2::Vector3(0, 0, 1), state.yaw);

    // Rotation from quaternion & translation from vector
    tf2::Transform transform(
        quaternion, tf2::Vector3(state.x, state.y, state.z));

    tf2::toMsg(transform, pose_stamped.pose);

    robot_state_publisher_[robot->idx].publish(pose_stamped);
  }
}

void PfucltPublisher::publishTargetState()
{
  for (const auto& target : pfuclt_->targets_) {
    geometry_msgs::PointStamped& point_stamped = estimate_msg_.targets_estimate[target->idx];
    const particle::TargetSubparticle& state = pfuclt_->state_->states.targets[target->idx];
    const std::vector<bool>& visibility = pfuclt_->state_->targets_visibility[target->idx];

    estimate_msg_.targets_visibility[target->idx] = std::any_of(visibility.begin(), visibility.end(),
                                                                [](bool t) { return t == true; });

    point_stamped.header.stamp = target->last_motion;
    point_stamped.header.frame_id = "world";
    point_stamped.point.x = state.x;
    point_stamped.point.y = state.y;
    point_stamped.point.z = state.z;
    target_state_publisher_[target->idx].publish(point_stamped);
  }
}

void PfucltPublisher::publishEstimate()
{
  // estimate_msg_ has been built in other methods (publishRobotState and publishTargetState)
  estimate_msg_.iteration_time_ms = pfuclt_->time_iteration_ms_;
  estimate_publisher_.publish(estimate_msg_);
}

void PfucltPublisher::run()
{
  publishRobotParticles();
  publishTargetParticles();
  publishRobotState();
  publishTargetState();
  publishEstimate();
}

}  // namespace pfuclt::publisher
//
// Created by glawless on 9/4/18.
//

#include "robot.hpp"
#include <random>
#include <angles/angles.h>

namespace pfuclt::robot {

Robot::Robot(const uint id, particle::RobotSubParticles* p_subparticles, const map::LandmarkMap* p_map)
  : idx(id), name(Robot::name_prefix_ + std::to_string(idx+1)),
  generator_(rd_()), nh_("/robots/"+name), subparticles(p_subparticles), map(p_map) {

  getAlphas();

  odometry_sub_ = nh_.subscribe("/odometry", 100, &Robot::odometryCallback, this);
  target_sub_ = nh_.subscribe("/target", 100, &Robot::targetCallback, this);
  landmark_sub_ = nh_.subscribe("/landmark", 100, &Robot::landmarkCallback, this);
}

void Robot::odometryCallback(const clt_msgs::CustomOdometryConstPtr &msg) {
  odometry_cache_.emplace(odometry::fromRosCustomMsg(msg));
}

void Robot::processOdometryMeasurement(const odometry::OdometryMeasurement &odom) {
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
    // Retrieve oldest element from queue
    auto odom = odometry_cache_.front();
    odometry_cache_.pop();
    processOdometryMeasurement(odom);
  }
}

void Robot::targetCallback(const clt_msgs::MeasurementStampedConstPtr &msg) {
  //target_cache_.emplace(target_data::fromRosMsg(msg));
}

void Robot::processTargetMeasurement() {
  
}

void Robot::landmarkCallback(const clt_msgs::MeasurementArrayConstPtr &msg) {
  landmark_measurements_ = std::move(landmark::fromRosMsg(msg));
}

int Robot::landmarksUpdate(particle::WeightSubParticles & probabilities) {
  if (landmark_measurements_ == nullptr)
  {
    return -1;
  }

  auto number_seen = std::count_if(landmark_measurements_->measurements.begin(), landmark_measurements_->measurements.end(),[](const auto& m) {
    return m.seen;
  });

  for (const auto& measurement: landmark_measurements_->measurements)
  {
    auto cov(std::move(landmark::uncertaintyModel(measurement)));
    
  }


  landmark_measurements_.reset();
  return number_seen;



/*
  std::vector<double> weights(subparticles->size(), 1.0);


  // Retrieve oldest element of queue
  auto lk = landmark_cache_.front();
  landmark_cache_.pop();  // Observation in robot frame

  for (auto& landmark: lk.measurements) {

    std::array<double,2> landmark_global{map->landmarks[landmark.id].x,
                                         map->landmarks[landmark.id].y};

    //TODO Convert landmark position in robot frame (range,bearing) to global frame (x,y)
    

    for(auto& particle: *subparticles) {

      // Robot position
      std::array<double,2> robot_position{particle.x,
                                          particle.y};

      // TODO Calculate error in observation
      std::array<double,2> error;

      // Update weight for this particle
      
    }
  }
*/
}

} // namespace pfuclt::robot
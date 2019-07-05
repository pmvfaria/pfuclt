//
// Created by glawless on 9/4/18.
//

#include "robot.hpp"
#include <random>
#include <angles/angles.h>

namespace pfuclt::robot {

Robot::Robot(const uint id, particle::RobotSubParticles* p_subparticles, const map::LandmarkMap* p_map)
  : idx(id), name(Robot::name_prefix_ + std::to_string(idx+1)),
  generator_(rd_()), nh_("/robots/"+name),
  odometry_sub_(nh_.subscribe("/odometry", 100, &Robot::odometryCallback, this)),
  target_sub_(nh_.subscribe("/target", 100, &Robot::targetCallback, this)),
  landmark_sub_(nh_.subscribe("/landmark", 100, &Robot::landmarkCallback, this)),
  subparticles(p_subparticles), map(p_map) {

  this->initialize();
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

void Robot::motionModel() {
  while (!odometry_cache_.empty())
  {
    // Retrieve oldest element from queue
    auto odom = odometry_cache_.front();
    odometry_cache_.pop();
    processOdometryMeasurement(odom);
  }
}

void Robot::targetCallback(const clt_msgs::MeasurementStampedConstPtr& msg) {
  //target_cache_.emplace(target_data::fromRosMsg(msg));
}

void Robot::processTargetMeasurement() {
  
}

void Robot::landmarkCallback(const clt_msgs::MeasurementArrayConstPtr &msg) {
  landmark_measurements_ = std::move(landmark::fromRosMsg(msg));
}

int Robot::landmarksUpdate(particle::WeightSubParticles& probabilities) {
  if (landmark_measurements_ == nullptr)
  {
    return -1;
  }

  for (const auto& m: landmark_measurements_->measurements)
  {
    if (m.seen)
    {
      auto cov(landmark::uncertaintyModel(m));

      std::for_each(subparticles->begin(), subparticles->end(), [this, idx=0, &m, &cov, &probabilities] (const auto &p) mutable -> void
      {
        // All to robot frame in cartesian coordinates
        double m_x {m.range * cos(m.bearing + p.theta)};
        double m_y {m.range * sin(m.bearing + p.theta)};
        double lm_local_x {this->map->landmarks[m.id].x - p.x};
        double lm_local_y {this->map->landmarks[m.id].y - p.y};

        double m_length {sqrt(m_x * m_x + m_y * m_y)};
        double m_angle {atan2(m_y, m_x)};
        double lm_length {sqrt(lm_local_x * lm_local_x + lm_local_y * lm_local_y)};
        double lm_angle {atan2(lm_local_y, lm_local_x)};

        double z_length {m_length - lm_length};
        double z_angle {m_angle - lm_angle};

        //TODO confirm model
        // Bivariate Gaussian
        double num_length {z_length * z_length / (2.0 * cov.dd * cov.dd)};
        double num_angle {z_angle * z_angle / (2.0 * cov.pp * cov.pp)};
        double numerator {exp(-1.0 * (num_length + num_angle))};
        double denominator {2.0 * M_PI * cov.dd * cov.pp};

        probabilities[idx] = numerator / denominator;
        ++idx;
      });
    }
  }

  return std::count_if(landmark_measurements_->measurements.begin(), landmark_measurements_->measurements.end(),[](const auto& m) {
    return m.seen;
  });
}

void Robot::clearLandmarkMeasurements()
{
  if (landmark_measurements_ != nullptr)
  {
    landmark_measurements_.reset();
  }
}


} // namespace pfuclt::robot
#include "pfuclt/robot.hpp"

#include <cmath>
#include <algorithm>

#include "angles/angles.h"


namespace pfuclt::robot {

using namespace pfuclt;

Robot::Robot(const int id,
             particle::RobotSubparticles* robot_subparticles,
             pfuclt::state::State* state,
             const map::LandmarkMap* landmark_map)
  : idx(id), name(Robot::name_prefix_ + std::to_string(idx+1)),
  subparticles(robot_subparticles), generator_(rd_()), 
  map_(landmark_map), state_(state), nh_("/robots/"+name),
  odometry_sub_(nh_.subscribe("odometry", 10, &Robot::odometryCallback, this)),
  target_sub_(nh_.subscribe("target_measurements", 10, &Robot::targetCallback, this)),
  landmark_sub_(nh_.subscribe("landmark_measurements", 10, &Robot::landmarkCallback, this))
{
  initialize();
}

void Robot::odometryCallback(const clt_msgs::CustomOdometryConstPtr& msg)
{
  odometry_cache_.emplace(sensor::odometry::fromRosCustomMsg(msg));
}

void Robot::targetCallback(const clt_msgs::MeasurementArrayConstPtr& msg)
{
  target_measurements_ = sensor::measurement::fromRosMsg(msg);

  for (const auto& m : target_measurements_->measurements) {
    bool visible = std::any_of(state_->targets_visibility[m.id].begin(),
                               state_->targets_visibility[m.id].end(),
                               [](bool r) { return r == true; });

    if (!visible && m.seen == true)
      state_->targets_found[m.id] = idx;
    state_->targets_visibility[m.id][idx] = m.seen;
  }
}

void Robot::landmarkCallback(const clt_msgs::MeasurementArrayConstPtr& msg)
{
  landmark_measurements_ = sensor::measurement::fromRosMsg(msg);
}

void Robot::processOdometryMeasurement(const sensor::odometry::OdometryMeasurement& odom)
{
  // Robot motion model (modified from Probabilistic Robotics book)
  std::normal_distribution<double>
      yaw_rand(
          odom.delta_yaw,
          alphas_[0] * std::abs(odom.delta_yaw) + alphas_[1] * odom.delta_translation),
      trans_rand(
          odom.delta_translation,
          alphas_[2] * odom.delta_translation + alphas_[3] * std::abs(odom.pitch + odom.delta_yaw));

  for (auto& subparticle : *subparticles) {
    const double sample_translation = trans_rand(generator_);
    subparticle.yaw = angles::normalize_angle(subparticle.yaw + yaw_rand(generator_));
    subparticle.x += sample_translation * cos(subparticle.yaw) * std::abs(cos(odom.pitch));
    subparticle.y += sample_translation * sin(subparticle.yaw) * std::abs(cos(odom.pitch));
    subparticle.z += sample_translation * sin(odom.pitch);
  }
}

void Robot::processLandmarkMeasurement(const sensor::measurement::Measurement& measurement,
                                       particle::WeightSubparticles& probabilities)
{
  static const double pow_2pi_15 = 15.749609945722419;

  if (measurement.seen) {
    sensor::measurement::Likelihood cov(sensor::measurement::uncertaintyModel(measurement));

    double denominator {pow_2pi_15 * cov.dd * cov.aa * cov.ee};

    int p = 0;
    for (const auto& subparticle : *subparticles) {

      // In robot frame
      // x' = xcos(a)+ysin(a)
      // y' = -xsin(a)+ycos(a)
      double expected_distance_x {(map_->landmarks[measurement.id].x - subparticle.x) * std::cos(subparticle.yaw) +
                                  (map_->landmarks[measurement.id].y - subparticle.y) * std::sin(subparticle.yaw)};
      double expected_distance_y {-(map_->landmarks[measurement.id].x - subparticle.x) * std::sin(subparticle.yaw) +
                                  (map_->landmarks[measurement.id].y - subparticle.y) * std::cos(subparticle.yaw)};
      double expected_distance_z {map_->landmarks[measurement.id].height - subparticle.z};

      double expected_distance {std::sqrt(expected_distance_x * expected_distance_x +
                                          expected_distance_y * expected_distance_y + 
                                          expected_distance_z * expected_distance_z)};
      double expected_azimuth {std::atan2(expected_distance_y, expected_distance_x)};
      double expected_elevation {std::asin(expected_distance_z / expected_distance)};

      double error_distance {measurement.distance - expected_distance};
      double error_azimuth {angles::normalize_angle(measurement.azimuth - expected_azimuth)};
      double error_elevation {angles::normalize_angle(measurement.elevation - expected_elevation)};
      
      // Multivariate Gaussian Distribution
      double numerator_distance {error_distance * error_distance / (cov.dd * cov.dd)};
      double numerator_azimuth {error_azimuth * error_azimuth / (cov.aa * cov.aa)};
      double numerator_elevation {error_elevation * error_elevation / (cov.ee * cov.ee)};
      double numerator {std::exp(-0.5 * (numerator_distance + numerator_azimuth + numerator_elevation))};

      probabilities[p] *= numerator / denominator;
      ++p;
    }
  }
}

void Robot::processTargetMeasurement(const sensor::measurement::Measurement& measurement,
                                     const int& robot_particle_idx,
                                     const particle::TargetSubparticle& target_subparticle,
                                     double& probabilities)
{
  static const double pow_2pi_15 = 15.749609945722419;

  if (measurement.seen) {
    sensor::measurement::Likelihood cov(sensor::measurement::uncertaintyModel(measurement));

    const auto& robot_subparticle = (*subparticles)[robot_particle_idx];

    // In robot frame
    // x' = xcos(a)+ysin(a)
    // y' = -xsin(a)+ycos(a)
    double expected_distance_x {(target_subparticle.x - robot_subparticle.x) * std::cos(robot_subparticle.yaw) +
                                (target_subparticle.y - robot_subparticle.y) * std::sin(robot_subparticle.yaw)};
    double expected_distance_y {-(target_subparticle.x - robot_subparticle.x) * std::sin(robot_subparticle.yaw) +
                                (target_subparticle.y - robot_subparticle.y) * std::cos(robot_subparticle.yaw)};
    double expected_distance_z {target_subparticle.z - robot_subparticle.z};

    double expected_distance {std::sqrt(expected_distance_x * expected_distance_x +
                                        expected_distance_y * expected_distance_y +
                                        expected_distance_z * expected_distance_z)};
    double expected_azimuth {std::atan2(expected_distance_y, expected_distance_x)};
    double expected_elevation {std::asin(expected_distance_z / expected_distance)};

    double error_distance {measurement.distance - expected_distance};
    double error_azimuth {angles::normalize_angle(measurement.azimuth - expected_azimuth)};
    double error_elevation {angles::normalize_angle(measurement.elevation - expected_elevation)};

    // Multivariate Gaussian Distribution
    double numerator_distance {error_distance * error_distance / (cov.dd * cov.dd)};
    double numerator_azimuth {error_azimuth * error_azimuth / (cov.aa * cov.aa)};
    double numerator_elevation {error_elevation * error_elevation / (cov.ee * cov.ee)};
    double numerator {std::exp(-0.5 * (numerator_distance + numerator_azimuth + numerator_elevation))};
    double denominator {pow_2pi_15 * cov.dd * cov.aa * cov.ee};

    probabilities *= numerator / denominator;
  }
}

bool Robot::landmarksUpdate(particle::WeightSubparticles& probabilities)
{
  if (landmark_measurements_ == nullptr)
    return false;

  for (const auto& measurement : landmark_measurements_->measurements)
    processLandmarkMeasurement(measurement, probabilities);

  bool landmark_seen = std::any_of(landmark_measurements_->measurements.begin(),
                                   landmark_measurements_->measurements.end(),
                                   [](const auto& l) { return l.seen; });

  return landmark_seen;
}

const sensor::measurement::Measurement* Robot::getTargetMeasurement(const int& target_id) const
{
  if (target_measurements_ == nullptr)
    return nullptr;

  const auto& ms = target_measurements_->measurements;
  
  const auto measurement = std::find_if(ms.begin(), ms.end(),
                                        [&target_id](const auto& m) { return m.id == target_id; });

  if (measurement != ms.end())
    return &(*measurement);
  else
    return nullptr;
}

void Robot::clearLandmarkMeasurements()
{
  if (landmark_measurements_ != nullptr)
    landmark_measurements_.reset();
}

void Robot::clearTargetMeasurements()
{
  if (target_measurements_ != nullptr)
    target_measurements_.reset();
}

bool Robot::motionModel()
{
  if (odometry_cache_.empty())
    return false;

  while (!odometry_cache_.empty()) {
    // Retrieve oldest element from queue
    const auto& odom = odometry_cache_.front();
    processOdometryMeasurement(odom);
    odometry_cache_.pop();
  }

  return true;
}
} // namespace pfuclt::robot

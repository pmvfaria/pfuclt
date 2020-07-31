#include "target.hpp"

#include <cmath>


namespace pfuclt::target {

using namespace pfuclt;

Target::Target(const int id, particle::TargetSubparticles* target_subparticles)
  : idx(id), name(Target::name_prefix_ + std::to_string(idx+1)),
    subparticles(target_subparticles), generator_(rd_())
{
  initialize();
}

void Target::motionModel()
{
  auto now = ros::Time::now();
  auto diff_sec = (now - last_motion).toSec();

  // Target motion model
  std::normal_distribution<double> target_accel(motion_mean_, motion_stddev_);
    
  for (auto& subparticle : *subparticles)
    for (int s = 0; s < particle::TargetSubparticle::number_states; ++s)
      subparticle[s] += 0.5 * target_accel(generator_) * std::pow(diff_sec, 2);

  last_motion = now;
}

void Target::observationModel(const particle::RobotSubparticle& robot_subparticle,
                              const sensor::measurement::Measurement& target_measurement)
{
  if (target_measurement.seen) {
    last_motion = ros::Time::now();

    // In world frame
    // x = x'cos(a)-y'sin(a)
    // y = x'sin(a)+y'cos(a)
    particle::TargetSubparticle target_subparticle;
    target_subparticle.x = robot_subparticle.x +
      target_measurement.distance * std::cos(target_measurement.azimuth) * std::cos(robot_subparticle.yaw) -
      target_measurement.distance * std::sin(target_measurement.azimuth) * std::sin(robot_subparticle.yaw);
    target_subparticle.y = robot_subparticle.y +
      target_measurement.distance * std::cos(target_measurement.azimuth) * std::sin(robot_subparticle.yaw) +
      target_measurement.distance * std::sin(target_measurement.azimuth) * std::cos(robot_subparticle.yaw);
    target_subparticle.z = robot_subparticle.z +
      target_measurement.distance * std::sin(target_measurement.elevation);

    std::uniform_real_distribution<double> dist(-0.5, 0.5);

    for (uint p = 0; p < (*subparticles).size() / 2; ++p)
      for (int s = 0; s < (*subparticles)[0].number_states; ++s)
        (*subparticles)[p][s] = target_subparticle[s] + dist(generator_);
  }
  else
    motionModel();
}

void Target::computeStdDev(const pfuclt::particle::WeightSubparticles& weights)
{
  static const double lost_stddev = param_stddev_ * 10.0;

  double sum = std::accumulate(weights.begin(),
                               weights.begin() + (weights.end() - weights.begin()) / 10,
                               0.0);
  if (sum > 1E-10 || motion_stddev_ == lost_stddev)
    motion_stddev_ = param_stddev_;
  else
    motion_stddev_ = lost_stddev;
}

} // namespace pfuclt::target
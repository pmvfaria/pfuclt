#ifndef PFUCLT_TARGET_HPP
#define PFUCLT_TARGET_HPP

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <random>
#include <queue>

#include <clt_msgs/Measurement.h>

#include "../particle/particles.hpp"
#include "../sensor/target_data.hpp"

namespace pfuclt::target {

using namespace ::pfuclt::sensor;

using generator_type = std::mt19937;

class Target {

 public:
  // id of this target - they should start at 0
  const uint idx;

  // name of this target - should end with a number
  const std::string name;

 private:
  std::random_device rd_{};
  generator_type generator_;

  static constexpr auto name_prefix_ = "target";

  // scoped namespace
  ::ros::NodeHandle nh_;

  // subscriber and queue to take target messages
  ros::Subscriber target_sub_;
  std::queue<target_data::TargetMeasurement> target_cache_;

 public:
  // pointer to this target's sub-particles
  particle::TargetSubParticles *subparticles;

 private:
  void targetCallback(const clt_msgs::MeasurementConstPtr&);
  void processTargetMeasurement(const clt_msgs::MeasurementConstPtr&);


 public:
  Target() = delete;
  Target(const Target &) = delete; // no copy
  Target(Target &&) = delete; // no move
  Target& operator=(const Target &) = delete; // no copy assign
  Target& operator=(Target &&) = delete; // no move assign

  /**
   * Constructor
   * @param idx the id of this target (usually should start at 0)
   * @param subparticles pointer to the subparticles of these target in a set of particles
   * @param odometry_cb_queue pointer to a callback queue to associate with the odometry subscriber
   */
  Target(uint id, particle::TargetSubParticles* p_subparticles, ros::CallbackQueue* target_cb_queue);

};

} // namespace pfuclt::target

#endif //PFUCLT_TARGET_HPP

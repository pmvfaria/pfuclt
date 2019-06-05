#include "target.hpp"
#include <random>

namespace pfuclt::target {

Target::Target(const uint id, particle::TargetSubParticles* p_subparticles, ros::CallbackQueue* target_cb_queue)
  : idx(id), name(Target::name_prefix_ + std::to_string(idx+1)),
  generator_(rd_()), nh_("/targets/"+name), subparticles(p_subparticles) {

  nh_.setCallbackQueue(target_cb_queue);
  target_sub_ = nh_.subscribe("target", 100, &Target::targetCallback, this);

}

void Target::targetCallback(const clt_msgs::MeasurementConstPtr &msg) {
  target_cache_.emplace(target_data::fromRosMsg(msg));
}


void Target::processTargetMeasurement(const clt_msgs::MeasurementConstPtr &msg) {

  // Target model
  
}

} // namespace pfuclt::target
#include "ros/debug.hpp"

#include <sstream>
#include <algorithm>
#include <functional>
#include <cmath>

#include <boost/bind.hpp>

#include "tf/transform_datatypes.h"

#include "particle/subparticle.hpp"
#include "pfuclt/pfuclt.hpp"


namespace pfuclt::debug {

using namespace pfuclt;

Error::Error(const algorithm::PFUCLT& pfuclt)
  : robot_gt_sub_(pfuclt.num_robots), target_gt_sub_(pfuclt.num_targets), counter_(0.0)
{
  pfuclt_ = &pfuclt;

  ground_truth_.robots.resize(pfuclt_->num_robots);
  ground_truth_.targets.resize(pfuclt_->num_targets);

  error_.robots.resize(pfuclt_->num_robots);
  error_.targets.resize(pfuclt_->num_targets);
  sum_error_.robots.resize(pfuclt_->num_robots);
  sum_error_.targets.resize(pfuclt_->num_targets);
  avg_error_.robots.resize(pfuclt_->num_robots);
  avg_error_.targets.resize(pfuclt_->num_targets);

  for (particle::RobotSubparticle& robot : sum_error_.robots)
    for (int s = 0; s < robot.number_states; ++s)
      robot[s] = 0.0;
  for (particle::TargetSubparticle& target : sum_error_.targets)
    for (int s = 0; s < target.number_states; ++s)
      target[s] = 0.0;

  std::string topic;
  for (int i = 0; i < pfuclt_->num_robots; ++i) {
    topic = "robot" + std::to_string(i+1) + "/sim_pose";
    robot_gt_sub_[i] = rnh_.subscribe<geometry_msgs::PoseStamped>(
          topic, 10, boost::bind(&Error::robotGtCallback, this, _1, i));
  }
  for (int i = 0; i < pfuclt_->num_targets; ++i) {
    topic = "target" + std::to_string(i+1) + "/sim_pose";
    target_gt_sub_[i] = tnh_.subscribe<geometry_msgs::PointStamped>(
          topic, 10, boost::bind(&Error::targetGtCallback, this, _1, i));
  }
}

void Error::robotGtCallback(const geometry_msgs::PoseStampedConstPtr& msg, const int& idx)
{
  ground_truth_.robots[idx].x = msg->pose.position.x;
  ground_truth_.robots[idx].y = msg->pose.position.y;
  ground_truth_.robots[idx].z = msg->pose.position.z;

  tf::Quaternion q(msg->pose.orientation.x,
                   msg->pose.orientation.y,
                   msg->pose.orientation.z,
                   msg->pose.orientation.w);
  tf::Matrix3x3 mat(q);
  double roll, pitch, yaw;
  mat.getRPY(roll, pitch, yaw);

  ground_truth_.robots[idx].yaw = yaw;
}

void Error::targetGtCallback(const geometry_msgs::PointStampedConstPtr& msg, const int& idx)
{
  ground_truth_.targets[idx].x = msg->point.x;
  ground_truth_.targets[idx].y = msg->point.y;
  ground_truth_.targets[idx].z = msg->point.z;
}

void Error::computeError()
{
  const state::States state = pfuclt_->getState();

  for (int r = 0; r < pfuclt_->num_robots; r++)
    for (int i = 0; i < state.robots[r].number_states; i++)
      error_.robots[r][i] = ground_truth_.robots[r][i] - state.robots[r][i];

  for (int t = 0; t < pfuclt_->num_targets; t++)
    for (int i = 0; i < state.targets[t].number_states; i++)
      error_.targets[t][i] = ground_truth_.targets[t][i] - state.targets[t][i];
}

void Error::computeAvgError()
{
  counter_++;

  for (uint r = 0; r < avg_error_.robots.size(); ++r)
    for (int s = 0; s < avg_error_.robots[r].number_states; ++s) {
      sum_error_.robots[r][s] += std::abs(error_.robots[r][s]);
      avg_error_.robots[r][s] = sum_error_.robots[r][s] / counter_;
    }

  for (uint t = 0; t < avg_error_.targets.size(); ++t)
    for (int s = 0; s < avg_error_.targets[t].number_states; ++s) {
      sum_error_.targets[t][s] += std::abs(error_.targets[t][s]);
      avg_error_.targets[t][s] = sum_error_.targets[t][s] / counter_;
    }
}

void Error::printError() const
{
  std::ostringstream debug;
  debug << "\nErrors:";
  for (int r = 0; r < pfuclt_->num_robots; ++r) {
    debug << "\nRobot" << r+1 << ":";
    for (int i = 0; i < error_.robots[r].number_states; ++i)
      debug << " " << error_.robots[r][i];
  }
  for (int t = 0; t < pfuclt_->num_targets; ++t) {
    debug << "\nTarget" << t+1 << ":";
    for (int i = 0; i < error_.targets[t].number_states; ++i)
      debug << " " << error_.targets[t][i];
  }

  ROS_DEBUG("%s", debug.str().c_str());
}

void Error::printAvgError() const
{
  std::ostringstream debug;
  debug << "\nAverage Errors:";
  for (int r = 0; r < pfuclt_->num_robots; ++r) {
    debug << "\nRobot" << r+1 << ":";
    for (int i = 0; i < avg_error_.robots[r].number_states; ++i)
      debug << " " << avg_error_.robots[r][i];
  }
  for (int t = 0; t < pfuclt_->num_targets; ++t) {
    debug << "\nTarget" << t+1 << ":";
    for (int i = 0; i < avg_error_.targets[t].number_states; ++i)
      debug << " " << avg_error_.targets[t][i];
  }

  ROS_DEBUG("%s", debug.str().c_str());
}

void Error::printGT() const
{
  std::ostringstream debug;
  debug << "\nGround Truth:";
  for (int r = 0; r < pfuclt_->num_robots; ++r) {
    debug << "\nRobot" << r+1 << ":";
    for (int i = 0; i < ground_truth_.robots[r].number_states; ++i)
      debug << " " << ground_truth_.robots[r][i];
  }
  for (int t = 0; t < pfuclt_->num_targets; ++t) {
    debug << "\nTarget" << t+1 << ":";
    for (int i = 0; i < ground_truth_.targets[t].number_states; ++i)
      debug << " " << ground_truth_.targets[t][i];
  }
  
  ROS_DEBUG("%s", debug.str().c_str());
}

} // namespace pfuclt::debug

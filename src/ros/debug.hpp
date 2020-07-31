#ifndef PFUCLT_ROS_DEBUG_HPP
#define PFUCLT_ROS_DEBUG_HPP

#include <vector>
#include <string>

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PointStamped.h"

#include "thirdparty/stopwatch/StopWatch.h"

#include "pfuclt/state.hpp"


namespace pfuclt::algorithm {
    class PFUCLT;
}

namespace pfuclt::debug {

class Error
{
 public:
  Error(const pfuclt::algorithm::PFUCLT& pfuclt);

  /**
   * @brief Calculate error between ground truth and estimated pose
   */
  void computeError();

  /**
   * @brief Calculate average error between ground truth and estimated pose,
   * from the beginning of the algorithm until the current time
   */
  void computeAvgError();

  /**
   * @brief Print in terminal the errors computed in computeError()
   * @details To print, debug flag has to be on
   */
  void printError() const;

  /**
   * @brief Print in terminal the average errors computed in computeAvgError()
   * @details To print, debug flag has to be on
   */
  void printAvgError() const;

  /**
   * @brief Print in terminal the ground truth of both the robots and targets
   * @details To print, debug flag has to be on
   */
  void printGT() const;


 private:
  const pfuclt::algorithm::PFUCLT* pfuclt_;

  ros::NodeHandle rnh_{"/robots"};
  ros::NodeHandle tnh_{"/targets"};

  std::vector<ros::Subscriber> robot_gt_sub_;
  std::vector<ros::Subscriber> target_gt_sub_;

  pfuclt::state::States ground_truth_, error_, sum_error_, avg_error_;

  double counter_;

  void robotGtCallback(const geometry_msgs::PoseStampedConstPtr& msg, const int& idx);
  void targetGtCallback(const geometry_msgs::PointStampedConstPtr& msg, const int& idx);
};

} // namespace pfuclt::debug

#endif // PFUCLT_ROS_DEBUG_HPP
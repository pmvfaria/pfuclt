#ifndef PFUCLT_ROS_PFUCLT_PUBLISHER_HPP
#define PFUCLT_ROS_PFUCLT_PUBLISHER_HPP

#include <vector>

#include "ros/ros.h"

#include "clt_msgs/Estimate.h"

namespace pfuclt::algorithm {
    class PFUCLT;
}

namespace pfuclt::publisher {

/**
 * @brief The publisher class - implents the ROS publishers for the particle
 * filter class (PFUCLT)
 */
class PfucltPublisher
{
 public:
  /**
   * @brief Constructor of the PfucltPublisher class
   * @param pfuclt Pointer to the particle filter
   */
  PfucltPublisher(pfuclt::algorithm::PFUCLT* pfuclt);

  /**
   * @brief Publish all the necessary information for the rviz visualization
   */
  void run();

 private:
  // Pointer to particle filter
  pfuclt::algorithm::PFUCLT* pfuclt_;

  ros::NodeHandle nh_{""};

  // Target
  std::vector<ros::Publisher> target_state_publisher_;
  std::vector<ros::Publisher> target_particles_publisher_;

  // Robot
  std::vector<ros::Publisher> robot_state_publisher_;
  std::vector<ros::Publisher> robot_particles_publisher_;

  ros::Publisher estimate_publisher_;
  
  clt_msgs::Estimate estimate_msg_;

  /**
   * @brief A series of PoseArray messages, containing the subparticles
   * for each robot are published.
   */
  void publishRobotParticles();

  /**
   * @brief A series of PointCloud messages, containing the subparticles
   * for each target are published.
   */
  void publishTargetParticles();

  /**
   * @brief Publish the state (pose) of each robot and build part of
   * estimate_msg_
   */
  void publishRobotState();

  /**
   * @brief Publish the state (point) of each target and build part of
   * estimate_msg_
   */
  void publishTargetState();

  /**
   * @brief Publish a custom message with each robot's state and its
   * iteration time and each target's state and its visibility (estimate_msg_)
   */
  void publishEstimate();
};

} // namespace pfuclt::publisher

#endif  // PFUCLT_ROS_PFUCLT_PUBLISHER_HPP

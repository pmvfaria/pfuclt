#ifndef PFUCLT_PUBLISHER_H
#define PFUCLT_PUBLISHER_H

#include "../pfuclt/pfuclt.hpp"

#include <vector>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/PointCloud.h>
#include <clt_msgs/Estimate.h>
#include <clt_msgs/Particles.h>
#include <clt_msgs/SubParticles.h>
#include <clt_msgs/SubParticle.h>
#include <clt_msgs/GroundTruth.h>

namespace pfuclt::algorithm{
    class PFUCLT;
}

namespace pfuclt::publisher {

/**
 * @brief The publisher class - implents the ROS publishers necessary to the
 * particle filter class (PFUCLT)
 */
class PFUCLTPublisher {

public:
    /**
     * @brief Constructor of the PFUCLTPublisher class
     * @params pfuclt Reference to the particle filter
     */
    PFUCLTPublisher(::pfuclt::algorithm::PFUCLT& pfuclt);

private:
    ::pfuclt::algorithm::PFUCLT* pfuclt_;

    ros::Subscriber groundTruthSubscriber_;

    ros::Publisher particlesPublisher_;
    ros::Publisher estimatePublisher_;


    // Rviz visualization publishers

    // Target
    std::vector<ros::Publisher> targetEstimatedPointPublisher_;
    std::vector<ros::Publisher> targetGtPointPublisher_;
    std::vector<ros::Publisher> targetParticlesPublisher_;
    // Target observations publisher
    std::vector<ros::Publisher> targetObservationsPublisher_;

    // Robot
    std::vector<ros::Publisher> robotEstimatedPosePublisher_;
    std::vector<ros::Publisher> robotGtPosePublisher_;
    std::vector<ros::Publisher> robotParticlesPublisher_;
    // Broadcaster
    std::vector<tf2_ros::TransformBroadcaster> robotBroadcaster_;

    // Messages
    clt_msgs::Estimate estimate_;
    clt_msgs::Particles particles_;
    clt_msgs::GroundTruth groundTruth_;

    /**
     * @brief Build and publish message particles_ with all the robot, target and 
     * weight subparticles. A series of PoseArray and PointCloud messages for each
     * robot and target, respectively, are also published.
     */
    void publishParticles();

    /**
     * @brief Build message estimate_ for each robot with its estimated pose and
     * with which targets are visible. Publish the robot pose. The broadcast of
     * the robots coordinate frames to TF2 is also done.
     */
    void publishRobot();

    /**
     * @brief Build message estimate_ for each target with its estimated point
     * coordinates and with if it was seen by any robot. Publish the target point
     * coordinates.
     */
    void publishTarget();

    /**
     * @brief Publish the estimate message, estimate_, partially built in publishRobot
     * and publishTarget.
     */
    void publishEstimate();

    /**
     * @brief
     */
    void publishGt();

    /**
     * @brief Publish targets in rviz
     */
    void publishTargetsObservations();

    /**
     * @brief Function called when subscribing to ground truth data.
     */
    void groundTruthCallback(const clt_msgs::GroundTruth::ConstPtr&);
};

} // namespace pfuclt::publisher

#endif  //PFUCLT_PUBLISHER_H
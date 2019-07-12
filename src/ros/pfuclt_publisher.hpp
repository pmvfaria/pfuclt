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
 * @brief The publisher class - implents the ROS publishers necessary to the particle
 * filter class (PFUCLT)
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

    void publishParticles();

    void publishRobot();

    void publishTarget();

    void publishEstimate();

    void publishTargetObsrvations();

    void groundTruthCallback(const clt_msgs::GroundTruth::ConstPtr&);
};

} // namespace pfuclt::publisher

#endif  //PFUCLT_PUBLISHER_H
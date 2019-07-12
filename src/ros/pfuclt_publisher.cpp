#include "pfuclt_publisher.hpp"

namespace pfuclt::publisher {

PFUCLTPublisher::PFUCLTPublisher(::pfuclt::algorithm::PFUCLT& pfuclt)
    : targetEstimatedPointPublisher_(pfuclt.num_targets),
      targetParticlesPublisher_(pfuclt.num_targets),
      targetObservationsPublisher_(pfuclt.num_targets),
      robotEstimatedPosePublisher_(pfuclt.num_robots),
      robotParticlesPublisher_(pfuclt.num_robots),
      robotBroadcaster_(pfuclt.num_robots) {

    pfuclt_ = &pfuclt;

    // Subscribe to Ground Truth
    groundTruthSubscriber_ = pfuclt_->nh_.subscribe<clt_msgs::GroundTruth>(
                            "groundTruth", 100,
                            boost::bind(&PFUCLTPublisher::groundTruthCallback, this, _1));

    estimatePublisher_ = pfuclt_->nh_.advertise<clt_msgs::Estimate>("/estimate", 100);
    particlesPublisher_ = pfuclt_->nh_.advertise<clt_msgs::Particles>("/particles", 100);
    
    // Robots
    for (auto& robot : pfuclt_->robots_) {

        robotEstimatedPosePublisher_[robot->idx] = 
            pfuclt_->nh_.advertise<geometry_msgs::PoseStamped>(
                    "/" + robot->name + "/estimatedPose", 100);

        robotGtPosePublisher_[robot->idx] = 
            pfuclt_->nh_.advertise<geometry_msgs::PoseStamped>(
                    "/" + robot->name + "/gtPose", 100);

        robotParticlesPublisher_[robot->idx] =
            pfuclt_->nh_.advertise<geometry_msgs::PoseArray>(
                    "/" + robot->name + "/particles", 100);

        // Build estimate msg
        estimate_.robotEstimates.push_back(clt_msgs::RobotData());
        for (uint i = 0; i < (uint) pfuclt_->num_targets; i++)
            estimate_.robotEstimates[robot->idx].targetVisibility.push_back(false);
    }

    // Targets
    for (auto& target : pfuclt_->targets_) {

        targetEstimatedPointPublisher_[target->idx] =
            pfuclt_->nh_.advertise<geometry_msgs::PointStamped>(
                    "/" + target->name + "/estimatedPoint", 100);

        targetGtPointPublisher_[target->idx] =
            pfuclt_->nh_.advertise<geometry_msgs::PointStamped>(
                    "/" + target->name + "/gtPoint", 100);

        targetParticlesPublisher_[target->idx] =
            pfuclt_->nh_.advertise<sensor_msgs::PointCloud>(
                    "/" + target->name + "/particles", 100);

        targetObservationsPublisher_[target->idx] = 
            pfuclt_->nh_.advertise<visualization_msgs::Marker>(
                    "/" + target->name + "/observations", 100);

        // Build estimate msg
        estimate_.targetEstimates.push_back(clt_msgs::TargetData());
    }

}

void PFUCLTPublisher::publishParticles() {

    // Particles

    // Robot
    for (uint r = 0; r < (uint) pfuclt_->num_robots; r++) {
        particle::RobotSubParticles& robotSubParticles = pfuclt_->particles_->robots[r];

        for (uint s = 0; s < (uint) pfuclt_->num_particles; s++) {
            particles_.robotSubparticles[r].subparticles[s].subparticle.insert(
                    particles_.robotSubparticles[r].subparticles[s].subparticle.begin(),
                    {robotSubParticles[s].x, robotSubParticles[s].y, robotSubParticles[s].theta});
        }
    }
    // Target
    for (uint t = 0; t < (uint) pfuclt_->num_targets; t++) {
        particle::TargetSubParticles& targetSubParticles = pfuclt_->particles_->targets[t];

        for (uint s = 0; s < (uint) pfuclt_->num_particles; s++) {
            particles_.targetSubparticles[t].subparticles[s].subparticle.insert(
                    particles_.targetSubparticles[t].subparticles[s].subparticle.begin(),
                    {targetSubParticles[s].x, targetSubParticles[s].y, targetSubParticles[s].z});
        }
    }
    // Weight
    for (uint s = 0; s < (uint) pfuclt_->num_particles; s++) {
        particles_.weigthSubparticles.subparticles[s].subparticle.insert(
                particles_.weigthSubparticles.subparticles[s].subparticle.begin(),
                pfuclt_->particles_->weights[s]);
    }
    particlesPublisher_.publish(particles_);


    // Robot
    for (auto& robot : pfuclt_->robots_) {

        geometry_msgs::PoseArray msgRobotParticles;
        msgRobotParticles.header.stamp = robot->lastestMeasurementTime;
        msgRobotParticles.header.frame_id = "world";

        particle::RobotSubParticles& subparticles =
                        pfuclt_->particles_->robots[robot->idx];

        for (auto& subparticle : subparticles) {
            // Rotation around z axis with a magnitude of theta rad
            tf2::Quaternion quaternion(tf2::Vector3(0, 0, 1), subparticle.theta);
            // Rotation from quaternion & translation from vector
            tf2::Transform transform(quaternion, tf2::Vector3(subparticle.x,
                                                              subparticle.y,
                                                              subparticle.theta));
            geometry_msgs::Pose pose;
            tf2::toMsg(transform, pose);
            msgRobotParticles.poses.insert(msgRobotParticles.poses.begin(), pose);
        }

        robotParticlesPublisher_[robot->idx].publish(msgRobotParticles);
    }

    //Target
    for (auto& target : pfuclt_->targets_) {

        sensor_msgs::PointCloud msgTargetParticles;
        msgTargetParticles.header.stamp = ros::Time::now();
        msgTargetParticles.header.frame_id = "world";

        particle::TargetSubParticles& subparticles =
                        pfuclt_->particles_->targets[target->idx];
                        
        for (auto& subparticle : subparticles) {
            geometry_msgs::Point32 point;
            point.x = subparticle.x;
            point.y = subparticle.y;
            point.z = subparticle.z;

            msgTargetParticles.points.insert(msgTargetParticles.points.begin(), point);
        }

        targetParticlesPublisher_[target->idx].publish(msgTargetParticles);
    }
}

void PFUCLTPublisher::publishRobot() {

    for (auto& robot : pfuclt_->robots_) {

        estimate_.header.stamp = robot->lastestMeasurementTime;

        clt_msgs::RobotData robotData = estimate_.robotEstimates[robot->idx];
        state::RobotState& state = pfuclt_->state_->robots[robot->idx];

        tf2::Quaternion quaternion(tf2::Vector3(0, 0, 1), state.theta);
        tf2::Transform transform(quaternion, tf2::Vector3(state.x, state.y, state.theta));
        
        tf2::toMsg(transform, robotData.robotPose);

        /*// Target Visibility
        for (auto& target : pfuclt_->targets_) {
            robotData.targetVisibility[target->idx] = 
        }*/

        // TF2 Broadcast
        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header.stamp = robot->lastestMeasurementTime;
        transformStamped.header.frame_id = "world";
        transformStamped.child_frame_id = robot->name;
        transformStamped.transform = tf2::toMsg(transform);
        robotBroadcaster_[robot->idx].sendTransform(transformStamped);

        // Publish pose msg using previous TF
        geometry_msgs::PoseStamped estimatedPose;
        estimatedPose.header.stamp = transformStamped.header.stamp;
        estimatedPose.header.frame_id = transformStamped.child_frame_id;

        robotEstimatedPosePublisher_[robot->idx].publish(estimatedPose);
    }
}

void PFUCLTPublisher::publishTarget() {

    for (auto& target : pfuclt_->targets_) {

        state::TargetState& state = pfuclt_->state_->targets[target->idx];

        estimate_.targetEstimates[target->idx].header.frame_id = "world";

        estimate_.targetEstimates[target->idx].targetPoint.x = state.x;
        estimate_.targetEstimates[target->idx].targetPoint.y = state.y;
        estimate_.targetEstimates[target->idx].targetPoint.z = state.z;
        //estimate_.targetEstimates[target->idx].found = state.seen;

        // Publish point msg using previous TF
        geometry_msgs::PointStamped estimatedPoint;
        estimatedPoint.header.stamp = ros::Time::now();
        estimatedPoint.header.frame_id = "world";
        estimatedPoint.point.x = state.x;
        estimatedPoint.point.y = state.y;
        estimatedPoint.point.z = state.z;

        targetEstimatedPointPublisher_[target->idx].publish(estimatedPoint);
    }
}

void PFUCLTPublisher::publishEstimate() {

    // TODO Calculate Computation time

    estimatePublisher_.publish(estimate_);
}

void PFUCLTPublisher::publishTargetObsrvations() {

    for (auto& robot : pfuclt_->robots_) {

        visualization_msgs::Marker marker;

        marker.header.stamp = robot->lastestMeasurementTime;
        marker.header.frame_id = robot->name;
        
        // Setting the same namespace and id will overwrite the previous marker
        marker.ns = robot->name + "TargetObservations";
        marker.id = 0;

        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;

        // Arrow draw
        geometry_msgs::Point tail;
        tail.x = tail.y = tail.z = 0.0;
        // Point at index 0 - tail tip - is 0,0,0 because we're in the local frame
        marker.points.push_back(tail);
        // Point at index 1 - head - is the target pose in the local frame
        geometry_msgs::Point head;
        /*
        //head.x = obs.x;
        //head.y = obs.y;
        //head.z = obs.z;
        //marker.points.push_back(head);

        marker.scale.x = 0.01;
        marker.scale.y = 0.03;
        marker.scale.z = 0.05;

        // Colour
        marker.color.a = 1;
        if (obs.found)
            marker.color.r = marker.color.g = marker.color.b = 0.6;
        else {
            marker.color.r = 1.0;
            marker.color.g = marker.color.b = 0.0;
        }

        // Delete marker after 2 seconds
        marker.lifetime = ros::Duration(2);

        targetObservationsPublisher_.publish(marker);*/
    }
}

void PFUCLTPublisher::groundTruthCallback(const clt_msgs::GroundTruth::ConstPtr& msg) {

}

} // namespace pfuclt::publisher
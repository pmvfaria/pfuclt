#ifndef PFUCLT_PFUCLT_ROBOT_HPP
#define PFUCLT_PFUCLT_ROBOT_HPP

#include <vector>
#include <array>
#include <string>
#include <random>
#include <queue>
#include <memory>

#include "ros/ros.h"

#include "clt_msgs/CustomOdometry.h"
#include "clt_msgs/MeasurementArray.h"
#include "particle/subparticle.hpp"
#include "particle/particles.hpp"
#include "map/landmark_map.hpp"
#include "sensor/odometry_data.hpp"
#include "sensor/measurements_data.hpp"
#include "pfuclt/state.hpp"

namespace pfuclt::robot {

using generator_type = std::mt19937;

/**
 * @brief The Robot class - Has the common variables and methods of all robots,
 * and is the base class of any specialized robots who may derive from it.
 * All robots should be instances of this class
 */
class Robot
{
 public:
  // id of this robot - they should start at 0
  const int idx;

  // name of this robot - should end with a number
  const std::string name;

  // pointer to this robot's sub-particles
  pfuclt::particle::RobotSubparticles *subparticles;

  Robot() = delete;
  Robot(const Robot &) = delete; // no copy
  Robot(Robot &&) = delete; // no move
  Robot& operator=(const Robot &) = delete; // no copy assign
  Robot& operator=(Robot &&) = delete; // no move assign

  /**
   * @brief Constructor of Robot
   * @param id the id of this robot (usually should start at 0)
   * @param robot_subparticles pointer to the subparticles of these robot in a set of particles
   * @param landmark_map pointer to the the landmark map for reference
   */
  Robot(const int id,
        pfuclt::particle::RobotSubparticles* robot_subparticles,
        pfuclt::state::State* state,
        const pfuclt::map::LandmarkMap* landmark_map);

  /**
   * @brief Updates probabilities for each particle taking into account the landmark observations
   * and their known positions
   * @param probabilities vector of weights to be updated
   * @return Whether any landmark was seen by this robot
   */
  bool landmarksUpdate(pfuclt::particle::WeightSubparticles& probabilities);

  /**
   * @brief Retrieves target measurement with specified id
   * @param target_id Id of the target measurement pretended
   * @return Pointer to target measurement in target_measurements_ or null pointer
   */
  const pfuclt::sensor::measurement::Measurement* getTargetMeasurement(const int& target_id) const;

  /**
   * @brief Computes a probability for each particle given a target measurement
   * @param target_measurement Target measurement to be used in the computation of the probability
   * @param robot_particle_idx Index of robot subparticle to be used in the computation of the probability
   * @param target_subparticle Target subparticle to be used in the computation of the probability
   * @param probabilities Value to be updated with the computed probability
   */
  void processTargetMeasurement(const pfuclt::sensor::measurement::Measurement& target_measurement,
                                const int& robot_particle_idx,
                                const pfuclt::particle::TargetSubparticle& target_subparticle,
                                double& probabilities);

  /**
   * @brief After using all the landmark measurements to compute an estimate of a probability this
   * method is called to clear/reset the stored pointer to the landmark measurements
   */
  void clearLandmarkMeasurements();

  /**
   * @brief After using all the target measurements to compute an estimate of a probability this
   * method is called to clear/reset the stored pointer to the target measurements
   */
  void clearTargetMeasurements();


  /**
   * @brief Process all cached odometry messages, sampling the motion model for each particle
   * @return Boolean of whether the robot subparticles were updated (based on the odometry measurements)
   */
  bool motionModel();

  /**
   * @brief Computes the next standard deviation to be used in the robot predict step of the algorithm.
   * @param weights Weights associated with each particle
   * @details If the weights sum of the first 1/10 particles is smaller than 1E-10, the standard
   * deviation used in predictRobots() is increased for the next robot predict step.
   */
  void computeStdDev(const pfuclt::particle::WeightSubparticles& weights);


 private:
  std::random_device rd_{};
  generator_type generator_;

  static constexpr auto name_prefix_ = "robot";

  // pointer to landmark map
  const pfuclt::map::LandmarkMap* map_;

  // pointer to state
  pfuclt::state::State* state_;

  // Robot specific parameters that specify the noise in robot motion
  std::vector<double> alphas_;

  // scoped namespace
  ros::NodeHandle nh_;

  // subscriber and queue to take odometry messages
  ros::Subscriber odometry_sub_;
  std::queue<pfuclt::sensor::odometry::OdometryMeasurement> odometry_cache_;

  // subscriber and pointer to target measurements
  ros::Subscriber target_sub_;
  std::unique_ptr<pfuclt::sensor::measurement::Measurements> target_measurements_;

  // subscriber and pointer to landmark measurements
  ros::Subscriber landmark_sub_;
  std::unique_ptr<pfuclt::sensor::measurement::Measurements> landmark_measurements_;
  

  /**
   * @brief Loads the alphas from the parameter server
   * @details This method is called in the constructor
   */  
  void initialize();

  /**
   * @brief Event-driven method that should be called when
   * new odometry data is received
   * @param msg Message read from subscribed topic 
   */
  void odometryCallback(const clt_msgs::CustomOdometryConstPtr& msg);

  /**
   * @brief Event-driven method which should be called when
   * new landmark data is received
   * @param msg Message read from subscribed topic 
   */
  void landmarkCallback(const clt_msgs::MeasurementArrayConstPtr& msg);

  /**
   * @brief Event-driven method that should be called when
   * new target data is received
   * @param msg Message read from subscribed topic 
   */
  void targetCallback(const clt_msgs::MeasurementArrayConstPtr& msg);
  
  /**
   * @brief Uses robot motion model to estimate new particle's state for this robot
   * @param odometry_measurement odometry measurement retrieved in method odometryCallback
   * is used to calculate the new particle state
   */
  void processOdometryMeasurement(const pfuclt::sensor::odometry::OdometryMeasurement& odometry_measurement);

  /**
   * @brief Computes a probability for each particle given a landmark measurement
   * @param landmark_measurement landmark measurement retrieved in method landmarkCallback
   * is used to calculate the new particle state
   * @param weight_subparticles Values to be updated with the computed probabilities
   */
  void processLandmarkMeasurement(const pfuclt::sensor::measurement::Measurement& landmark_measurement,
                                  particle::WeightSubparticles & weight_subparticles);
};

} // namespace pfuclt::robot

#endif // PFUCLT_PFUCLT_ROBOT_HPP

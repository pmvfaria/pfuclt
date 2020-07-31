#include "pfuclt/pfuclt.hpp"

#include <numeric>
#include <algorithm>
#include <atomic>
#include <parallel/algorithm> // parallel for_each

#include "ros/globals.hpp"
#include "ros/debug.hpp"
#include "sensor/measurements_data.hpp"


namespace pfuclt::algorithm {

using namespace pfuclt;

PFUCLT::PFUCLT(const int self_robot_id)
  : self_robot_id(self_robot_id), rate_(15)
{
  // Get important parameters
  if (!pnh_.getParam("particles", num_particles))
    ROS_FATAL_STREAM("Particles parameter not found: " << pnh_.resolveName("particles"));
  if (!nh_.getParam("num_robots", num_robots))
    ROS_FATAL_STREAM("Number Robots parameter not found: " << pnh_.resolveName("num_robots"));
  if (!nh_.getParam("num_targets", num_targets))
    ROS_FATAL_STREAM("Number Targets parameter not found: " << pnh_.resolveName("num_targets"));

  ROS_INFO("Waiting for /clock");
  ros::Time::waitForValid();

  // PFUCLT initial time
  time_init_ = ros::Time::now();
  ROS_INFO("Init time set to %f", time_init_.toSec());

  // Get covariance coefficients
  getCovariances();

  // Get landmarks map
  num_landmarks = getLandmarkMap();
  ROS_ASSERT(num_landmarks > 0);

  ROS_INFO_STREAM("Added " << num_landmarks << " landmarks");
  ROS_INFO_STREAM(*map_);

  // Define per-robot per-particle persistent weight components
  weight_components_robots_ = std::vector<particle::WeightSubparticles>
                                    (num_robots, std::vector<double> (num_particles, 1.0));
  // Define per-target per-particle persistent weight components
  weight_components_targets_= std::vector<particle::WeightSubparticles>
                                    (num_targets, std::vector<double> (num_particles, 1.0));

  // Create and initialize particles
  particles_ = std::make_unique<particle::Particles>(num_particles, num_robots, num_targets);

  if (initializeParticles())
    ROS_INFO("Initialized robot and target particles using parameters");
  else
    ROS_INFO("Initialized robot particles using parameters and target particles randomly");

  ROS_INFO_STREAM(*particles_);

  // Create state
  state_ = std::make_unique<state::State>(particles_.get());
  ROS_INFO_STREAM("State estimate created");

  // Create robots
  robots_.reserve(num_robots);
  for (int r = 0; r < num_robots; ++r) {
    robots_.emplace_back(std::make_unique<robot::Robot>(r, &particles_->robots[r], state_.get(), map_.get()));
    ROS_INFO_STREAM("Robot created with index " << robots_[r]->idx << " and name " << robots_[r]->name);
  }

  // Create targets
  targets_.reserve(num_targets);
  for (int t = 0; t < num_targets; ++t) {
    targets_.emplace_back(std::make_unique<target::Target>(t, &particles_->targets[t]));
    ROS_INFO_STREAM("Target created with index " << targets_[t]->idx << " and name " << targets_[t]->name);
  }

  // Create publisher
  if (globals::publish) {
    publisher_ = std::make_unique<publisher::PfucltPublisher>(this);
    ROS_INFO_STREAM("Publisher class created");
  }
}

bool PFUCLT::predictRobots()
{
  // Keeps track if any of the robots updated its pose
  std::atomic<char> any_odometry {false};

  forEachRobot([&any_odometry, this] (auto &robot) -> void {
    char pose_updated = robot->motionModel();
    any_odometry.fetch_or(pose_updated);
  },
  __gnu_parallel::parallel_unbalanced);

  return any_odometry;
}

void PFUCLT::predictTargets()
{
  forEachTarget([this](auto &target) -> void {

    int& robot = this->state_->targets_found[target->idx];

    if (robot != -1) {

      const particle::RobotSubparticle& robot_subparticle =
          this->state_->states.robots[robot];

      const sensor::measurement::Measurement* target_measurement =
          this->robots_[robot]->getTargetMeasurement(target->idx);

      ROS_ASSERT_MSG(target_measurement != nullptr,
                    "Robot method getTargetMeasurement() should not return nullptr here");

      target->observationModel(robot_subparticle, *target_measurement);

      robot = -1;
    }
    else
      target->motionModel();
  },
  __gnu_parallel::parallel_unbalanced);
}

bool PFUCLT::fuseLandmarks()
{
  // Keeps track if any of the robots saw a landmark
  std::atomic<bool> any_measurement {false};

  forEachRobot([&any_measurement, this](auto& robot) -> void {
    auto& robot_weights = this->weight_components_robots_[robot->idx];    
    
    // Tracks the probabilty propagation associated to the robot state based on the landmarks observations
    particle::WeightSubparticles probabilities(this->num_particles, 1.0);

    bool landmark_seen = robot->landmarksUpdate(probabilities);

    if (landmark_seen) {
      any_measurement.store(true);

      robot->clearLandmarkMeasurements();

      // Get sorted weights indices
      std::vector<int> idx(this->num_particles);
      std::iota(idx.begin(), idx.end(), 0);
      std::sort(idx.begin(), idx.end(),
        [&probabilities](auto i1, auto i2) { return probabilities[i1] > probabilities[i2]; });

      // Sorting according to indices is O(n) using a O(n) copy of the subparticles
      auto& robot_subparticles = *robot->subparticles;
      const auto robot_subparticles_copy = robot_subparticles;

      for (int p = 0; p < this->num_particles; ++p) {
        robot_subparticles[p] = robot_subparticles_copy[idx[p]];
        robot_weights[p] = probabilities[idx[p]];
      }
    }
  },
  __gnu_parallel::parallel_unbalanced);

  return any_measurement.load();
}

bool PFUCLT::fuseTargets()
{
  // Keeps track if any of the robots saw a target
  std::atomic<bool> any_measurement {false};

  forEachTarget([&any_measurement, this](auto& target) -> void {
    auto& target_weights = this->weight_components_targets_[target->idx];

    std::vector<const sensor::measurement::Measurement*> measurements(this->num_robots);

    for (const auto& robot : this->robots_)
      measurements[robot->idx] = robot->getTargetMeasurement(target->idx);

    // Check if any of the robots detected the target
    if (std::any_of(measurements.begin(), measurements.end(),
                    [this](const auto& m) { return m != nullptr && m->seen == true; })) {

      any_measurement.store(true);

      int m_star;  // target's subparticle index with the maximum weight
      double max_weight;  // Keeps track of the maximum contributed weight of a target's subparticle

      // Particle set [1:M]
      for (int m = 0; m < this->num_particles; ++m) {

        max_weight = -1.0;
        #pragma omp parallel
        {
          int m_star_calc = -1;
          double max_weight_calc = -1.0;
          double probabilities = 1.0;

          #pragma omp for
          for (int p = m; p < this->num_particles; ++p) {
            for (const auto& robot : this->robots_)
              if (measurements[robot->idx] != nullptr)
                robot->processTargetMeasurement(*(measurements[robot->idx]), m, (*target->subparticles)[p], probabilities);

            if (probabilities > max_weight_calc) {
              max_weight_calc = probabilities;
              m_star_calc = p;
            }

            probabilities = 1.0;
          }

          #pragma omp critical
          {
            if (max_weight_calc > max_weight) {
              max_weight = max_weight_calc;
              m_star = m_star_calc;
            }
          }
        }

        // Swap particle m with m_star so that the most relevant target subparticles 
        // are at the lowest indexes (paired with the most relevant robot subparticles)
        std::swap((*target->subparticles)[m], (*target->subparticles)[m_star]);

        target_weights[m] = max_weight;
      }

      target->computeStdDev(target_weights);
    }
  },
  __gnu_parallel::parallel_unbalanced);

  for (auto& robot : this->robots_)
    robot->clearTargetMeasurements();

  return any_measurement.load();
}

void PFUCLT::updateWeights()
{
  // Reset weigths
  std::fill(particles_->weights.begin(), particles_->weights.end(), 1.0);

  for (int p = 0; p < num_particles; ++p) {

    for (int r = 0; r < num_robots; ++r)
      particles_->weights[p] *= weight_components_robots_[r][p];

    for (int t = 0; t < num_targets; ++t)
      particles_->weights[p] *= weight_components_targets_[t][p];
  }
}

void PFUCLT::resample()
{
  particles_->normalizeWeights(__gnu_parallel::parallel_unbalanced);
  
  auto& robots = particles_->robots;
  auto& targets = particles_->targets;
  auto& weights = particles_->weights;

  // Make copy of robots, targets and weight subparticles
  const auto robots_copy = robots;
  const auto targets_copy = targets;
  const auto weights_copy = weights;


  // Multinomial Resampling (modified)
  // See https://robotics.stackexchange.com/questions/479/particle-filters-how-to-do-resampling

  // Comulative sum of weigths
  particle::WeightSubparticles comulative_weigths(num_particles);
  std::partial_sum(weights.begin(), weights.end(), comulative_weigths.begin());

  std::uniform_real_distribution<> dist(0.0, 1.0);
  
  #pragma omp parallel for
  // Robot and target particles resampling
  for (int p = 0; p < num_particles; p++) {
    // Generate random number [0,1]
    double rand_num = dist(generator_);
    // Index of the comulative sum of weights vector in which the random number belongs
    int idx = std::upper_bound(comulative_weigths.begin(), comulative_weigths.end(), rand_num) - comulative_weigths.begin();

    // New particles
    for (int r = 0; r < num_robots; r++)
      robots[r][p] = robots_copy[r][idx];

    for (int t = 0; t < num_targets; t++)
      targets[t][p] = targets_copy[t][idx];

    weights[p] = weights_copy[idx];
  }
}

void PFUCLT::run()
{
  StopWatch watch;

  bool any_odometry, any_landmarks, any_targets;

  ROS_INFO_STREAM("Waiting for simulation to finish...");
  while (ros::ok()) {

    watch.Restart();

    any_odometry = predictRobots();

    predictTargets();

    any_landmarks = fuseLandmarks();

    any_targets = fuseTargets();

    if (any_landmarks || any_targets)
      updateWeights();

    if (any_odometry && std::accumulate(particles_->weights.begin(), particles_->weights.end(), 0.0) > 1e-10)
      resample();

    state_->estimateWeightedAvg();

    time_iteration_ms_ = watch.ElapsedMs();

    if (globals::publish)
      publisher_->run();

    ros::spinOnce();

    rate_.sleep();
  }
}

const state::States& PFUCLT::getState() const
{
  return this->state_->states;
}

void PFUCLT::forEachRobot(std::function<void(std::unique_ptr<robot::Robot>&)> const& f, const optional_parallel& tag)
{
  if(tag)
    __gnu_parallel::for_each(robots_.begin(), robots_.end(), f, tag.value());
  else
    std::for_each(robots_.begin(), robots_.end(), f);
}

void PFUCLT::forEachRobot(std::function<void(const std::unique_ptr<robot::Robot>&)> const& f, const optional_parallel& tag) const
{
  if(tag)
    __gnu_parallel::for_each(robots_.begin(), robots_.end(), f, tag.value());
  else
    std::for_each(robots_.begin(), robots_.end(), f);
}

void PFUCLT::forEachTarget(std::function<void(std::unique_ptr<target::Target>&)> const& f, const optional_parallel& tag)
{
  if(tag)
    __gnu_parallel::for_each(targets_.begin(), targets_.end(), f, tag.value());
  else
    std::for_each(targets_.begin(), targets_.end(), f);
}

void PFUCLT::forEachTarget(std::function<void(const std::unique_ptr<target::Target>&)> const& f, const optional_parallel& tag) const
{
  if(tag)
    __gnu_parallel::for_each(targets_.begin(), targets_.end(), f, tag.value());
  else
    std::for_each(targets_.begin(), targets_.end(), f);
}

} // namespace pfuclt::algorithm

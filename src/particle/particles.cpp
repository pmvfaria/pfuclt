#include "particles.hpp"

#include <parallel/numeric>
#include <parallel/algorithm>
#include <memory>
#include <numeric>
#include <algorithm>
#include <random>
#include <cassert> // assert - c++ library deprecated
#include <cmath> // pi
#include <stdexcept>
#include <sstream>

#include "ros/console.h"

namespace pfuclt::particle {
  
Particles::Particles(const int& num_particles, const int& num_robots, const int& num_targets)
  : num_particles(num_particles),
    robots(num_robots, RobotSubparticles(num_particles)),
    targets(num_targets, TargetSubparticles(num_particles)),
    weights(num_particles, 1.0 / num_particles) { }


Particles::Particles(Particles&& other) noexcept
  : num_particles(other.num_particles),
    robots(std::move(other.robots)),
    targets(std::move(other.targets)),
    weights(std::move(other.weights)) { }

Particles& Particles::operator=(Particles && other) noexcept
{
  num_particles = other.num_particles;
  robots = std::move(other.robots);
  targets = std::move(other.targets);
  weights = std::move(other.weights);

  return *this;
}


Particles& Particles::initialize(
    const std::vector<std::array<double[2], RobotSubparticle::number_states>>& robot_dist,
    const std::vector<std::array<double[2], TargetSubparticle::number_states>>& target_dist)
{
  assert(robot_dist.size() == robots.size() && target_dist.size() == targets.size());

  std::mt19937 generator(seed_);

  std::vector<std::unique_ptr<std::uniform_real_distribution<double>>> dist;

  // Robots subpaticles
  dist.reserve(RobotSubparticle::number_states);
  for (size_t r = 0; r < robot_dist.size(); ++r) {
    const auto &params = robot_dist[r];
    for (size_t s = 0; s < RobotSubparticle::number_states; ++s)
      dist.emplace_back(std::make_unique<std::uniform_real_distribution<double>>(params[s][0], params[s][1]));
                                    
    for (RobotSubparticle& robot : robots[r])
      for (size_t s = 0; s < RobotSubparticle::number_states; ++s)
        robot[s] = (*dist[s])(generator);

    dist.clear();
  }

  // Targets subpaticles
  dist.reserve(TargetSubparticle::number_states);
  for (size_t t = 0; t < target_dist.size(); ++t) {
    const auto &params = target_dist[t];
    for (size_t s = 0; s < TargetSubparticle::number_states; ++s)
      dist.emplace_back(std::make_unique<std::uniform_real_distribution<double>>(params[s][0], params[s][1]));
                                    
    for (TargetSubparticle& target : targets[t])
      for (size_t s = 0; s < TargetSubparticle::number_states; ++s)
        target[s] = (*dist[s])(generator);

    dist.clear();
  }

  return *this;
}


Particles& Particles::removeTarget(const std::vector<TargetSubparticles>::iterator& t)
{
  if (targets.empty())
    throw std::out_of_range("std::out_of_range exception: targets is empty");

  if (!(t >= targets.begin() && t < targets.end()))
    throw std::out_of_range("std::out_of_range exception: specified target does not exist");

  targets.erase(t);

  return *this;
}


Particles& Particles::removeTarget(const int& t)
{
  return removeTarget(targets.begin() + t);
}


Particles& Particles::addTarget() noexcept
{
  targets.emplace_back(TargetSubparticles(num_particles));

  return *this;
}


void Particles::printWeights(std::string pre) const
{
  std::ostringstream debug;
  debug << "Weights " << pre;
  for (int i = 0; i < num_particles; ++i)
    debug << weights[i] << " ";

  ROS_DEBUG("%s", debug.str().c_str());
}


Particles& Particles::normalizeWeights(const optional_parallel& tag)
{
  if (tag) { // parallel
    const auto sum_weights = __gnu_parallel::accumulate(weights.begin(), weights.end(), 0.0, tag.value());
    if (sum_weights <= 0.0)
      throw std::range_error("Weights have a sum of 0.0, not possible to normalize");
    else if (sum_weights != 1.0)
      __gnu_parallel::transform(weights.begin(), weights.end(), weights.begin(),
                                [sum_weights](auto& w) { return w / sum_weights; });
    return *this;
  }
  else { // sequential
    const auto sum = std::accumulate(weights.begin(), weights.end(), 0.0);
    if (sum <= 0.0)
      throw std::range_error("Weights have a sum of 0.0, not possible to normalize");
    else if(sum != 1.0)
      std::transform(weights.begin(), weights.end(), weights.begin(), [sum](auto& w) { return w / sum; });

    return *this;
  }
}


Particles& Particles::resize(const int& num_particles, const optional_parallel& tag)
{
  this->forEachRobot([num_particles](RobotSubparticles& r) { r.resize(num_particles); }, tag);
  this->forEachTarget([num_particles](TargetSubparticles& t) { t.resize(num_particles); }, tag);

  weights.resize(num_particles);

  return *this;
}


double Particles::sumOfWeights() const
{
  return std::accumulate(weights.begin(), weights.end(), 0.0);
}


WeightSubparticles Particles::getNormalizedWeightsCopy() const
{
  auto weights_copy = weights;

  const auto sum = std::accumulate(weights_copy.begin(), weights_copy.end(), 0.0);
  if (sum <= 0.0)
    throw std::range_error("Weights have a sum of 0.0, not possible to obtain normalized copy");
  else if(sum != 1.0)
    std::transform(weights_copy.begin(), weights_copy.end(), weights_copy.begin(), [sum](auto& w) { return w / sum; });

  return weights_copy;
}


void Particles::forEachRobot(std::function<void(RobotSubparticles&)> const& f, const optional_parallel& tag)
{
  if (tag)
    __gnu_parallel::for_each(robots.begin(), robots.end(), f, tag.value());
  else
    std::for_each(robots.begin(), robots.end(), f);
}

void Particles::forEachRobot(std::function<void(const RobotSubparticles&)> const& f, const optional_parallel& tag) const
{
  if (tag)
    __gnu_parallel::for_each(robots.begin(), robots.end(), f, tag.value());
  else
    std::for_each(robots.begin(), robots.end(), f);
}

void Particles::forEachTarget(std::function<void(TargetSubparticles&)> const& f, const optional_parallel& tag)
{
  if (tag)
    __gnu_parallel::for_each(targets.begin(), targets.end(), f, tag.value());
  else
    std::for_each(targets.begin(), targets.end(), f);
}

void Particles::forEachTarget(std::function<void(const TargetSubparticles&)> const& f, const optional_parallel& tag) const
{
  if (tag)
    __gnu_parallel::for_each(targets.begin(), targets.end(), f, tag.value());
  else
    std::for_each(targets.begin(), targets.end(), f);
}


std::ostream &operator<<(std::ostream& os, const Particles& p)
{
  return os << "Particle set with " << p.robots.size() << " robot sub-particles, " << p.targets.size()
            << " target sub-particles, with a total of " << p.weights.size() << " particles";
}

} // namespace pfuclt::particle

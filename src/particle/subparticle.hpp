#ifndef PFUCLT_SUBPARTICLE_HPP
#define PFUCLT_SUBPARTICLE_HPP

#include <vector>
#include <array>
#include <cassert>

namespace pfuclt::particle{

/** @template SubParticle
  * @brief Stores the number of states of the subparticle
  */
template <int STATES>
struct SubParticle{
  static constexpr std::size_t number_states{STATES};
};

/** @class RobotSubParticle
  * @brief Derived class from Subparticle class. Stores the position of a robot subparticle
  */
struct RobotSubParticle: public SubParticle<3>{
  RobotSubParticle() = default;
  RobotSubParticle(double x_a, double y_a, double theta_a) : SubParticle(), x(x_a), y(y_a), theta(theta_a) { };
  double x, y, theta;
};

/** @class TargetSubParticle
  * @brief Derived class from Subparticle class. Stores the position of a target subparticle
  */
struct TargetSubParticle: public SubParticle<3>{
  TargetSubParticle() = default;
  TargetSubParticle(double x_a, double y_a, double z_a) : SubParticle(), x(x_a), y(y_a), z(z_a){ };
  double x, y, z;
};

typedef std::vector<RobotSubParticle> RobotSubParticles;
typedef std::vector<TargetSubParticle> TargetSubParticles;
typedef std::vector<double> WeightSubParticles;

} // namespace pfuclt::particle

#endif //PFUCLT_SUBPARTICLE_HPP

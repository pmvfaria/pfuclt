#ifndef PFUCLT_PARTICLE_SUBPARTICLE_HPP
#define PFUCLT_PARTICLE_SUBPARTICLE_HPP

#include <vector>
	

namespace pfuclt::particle {

/** @template Subparticle
  * @brief Stores the number of states of the subparticle
  */
template <int STATES>
struct Subparticle
{
  static constexpr int number_states{STATES};
};

/** @class RobotSubparticle
  * @brief Derived class from Subparticle class. Stores the position of a robot subparticle
  */
struct RobotSubparticle: public Subparticle<4>
{
  RobotSubparticle() = default;
  RobotSubparticle(double x_a, double y_a, double z_a, double yaw_a);
    
  double x, y, z, yaw;

  double& operator[](int i);
  const double& operator[](int i) const;
};

/** @class TargetSubparticle
  * @brief Derived class from Subparticle class. Stores the position of a target subparticle
  */
struct TargetSubparticle: public Subparticle<3>
{
  TargetSubparticle() = default;
  TargetSubparticle(double x_a, double y_a, double z_a);

  double x, y, z;

  double& operator[](int i);
  const double& operator[](int i) const;
};

typedef std::vector<RobotSubparticle> RobotSubparticles;
typedef std::vector<TargetSubparticle> TargetSubparticles;
typedef std::vector<double> WeightSubparticles;

} // namespace pfuclt::particle

#endif // PFUCLT_PARTICLE_SUBPARTICLE_HPP

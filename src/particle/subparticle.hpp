#ifndef PFUCLT_SUBPARTICLE_HPP
#define PFUCLT_SUBPARTICLE_HPP

#include <vector>
#include <array>

namespace pfuclt::particle{

template <auto STATES>
class SubParticle{
 protected:
  std::array<double, STATES> p_;

 public:
  SubParticle() = default;
  explicit SubParticle(double init_value) { p_.fill(init_value); }
  SubParticle(std::initializer_list<double> init_values) { }

  std::array<double, STATES>& operator()() noexcept { return p_; }
};

class RobotSubParticle: public SubParticle<3>{
  // Inherit constructors
  using SubParticle::SubParticle;

 public:
  RobotSubParticle(double x, double y, double theta) : SubParticle({x, y, theta}) { };

  const double x() const { return p_[0]; };
  const double y() const { return p_[1]; };
  const double theta() const { return p_[2]; };
  double& x() { return p_[0]; };
  double& y() { return p_[1]; };
  double& theta() { return p_[2]; };
};

class TargetSubParticle: public SubParticle<3>{
  // Inherit constructors
  using SubParticle::SubParticle;

 public:
  TargetSubParticle(double x, double y, double z) : SubParticle({x, y, z}) { };

  const double x() const { return p_[0]; };
  const double y() const { return p_[1]; };
  const double theta() const { return p_[2]; };
  double& x() { return p_[0]; };
  double& y() { return p_[1]; };
  double& theta() { return p_[2]; };
};

typedef std::vector<RobotSubParticle> RobotSubParticles;
typedef std::vector<TargetSubParticle> TargetSubParticles;
typedef std::vector<double> WeightSubParticles;

} // namespace pfuclt::particle

#endif //PFUCLT_SUBPARTICLE_HPP

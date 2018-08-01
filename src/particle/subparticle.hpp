#ifndef PFUCLT_SUBPARTICLE_H
#define PFUCLT_SUBPARTICLE_H

#include <vector>
#include <array>

namespace pfuclt::particle{

class RobotSubParticle{
 private:
  std::array<double, 3> p_{0.0, 0.0, 0.0};

 public:
  RobotSubParticle() = default;
  explicit RobotSubParticle(double init_value) { p_.fill(init_value); }
  explicit RobotSubParticle(double init_values[3]) { std::copy(p_.begin(), p_.end(), init_values); }
  RobotSubParticle(double x, double y, double theta) : p_{x,y,theta} { };

//  std::array<double, 3> operator(){ return p_; };
  const double x() const { return p_[0]; };
  const double y() const { return p_[1]; };
  const double theta() const { return p_[2]; };
  double x() { return p_[0]; };
  double y() { return p_[0]; };
  double theta() { return p_[0]; };
};

class TargetSubParticle {
 private:
  std::array<double, 3> p_{0,0,0};

 public:
  TargetSubParticle() = default;
  explicit TargetSubParticle(double init_value) { p_.fill(init_value); }
  explicit TargetSubParticle(double init_values[3]) { std::copy(p_.begin(), p_.end(), init_values); }
  TargetSubParticle(double x, double y, double z) : p_{x,y,z} { };

  inline const double x() const { return p_[0]; };
  inline const double y() const { return p_[1]; };
  inline const double z() const { return p_[2]; };
  inline double x() { return p_[0]; };
  inline double y() { return p_[1]; };
  inline double z() { return p_[2]; };
};

typedef std::vector<RobotSubParticle> RobotSubParticles;
typedef std::vector<TargetSubParticle> TargetSubParticles;
typedef std::vector<double> WeightSubParticles;

} // namespace pfuclt::particle

#endif //PFUCLT_SUBPARTICLE_H

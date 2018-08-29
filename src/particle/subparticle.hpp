#ifndef PFUCLT_SUBPARTICLE_HPP
#define PFUCLT_SUBPARTICLE_HPP

#include <vector>
#include <array>
#include <cassert>

namespace pfuclt::particle{

template <auto STATES>
class SubParticle{
 protected:
  std::array<double, STATES> p_;

 public:
  SubParticle() = default;
  explicit SubParticle(double init_value) { p_.fill(init_value); }
  SubParticle(std::initializer_list<double> init_values) {
    std::size_t ii=0;
    for(auto &val: init_values){
      p_[ii++] = val;
    }
  }

  static constexpr std::size_t number_states{STATES};

  const std::array<double, STATES>& operator()() const noexcept { return p_; }
  std::array<double, STATES>& operator()() noexcept { return p_; }

  const double operator[](const int idx) const noexcept { assert(idx < STATES); return p_[idx]; }
  double& operator[](const int idx) noexcept { assert(idx < STATES); return p_[idx]; }
};

class RobotSubParticle: public SubParticle<3>{
  // Inherit constructors
  using SubParticle::SubParticle;

 public:
  RobotSubParticle(double x, double y, double theta) : SubParticle({x, y, theta}) { };

  inline const double x() const noexcept { return p_[0]; };
  inline const double y() const noexcept { return p_[1]; };
  inline const double theta() const noexcept { return p_[2]; };
  inline double& x() noexcept { return p_[0]; };
  inline double& y() noexcept { return p_[1]; };
  inline double& theta() noexcept { return p_[2]; };
};

class TargetSubParticle: public SubParticle<3>{
  // Inherit constructors
  using SubParticle::SubParticle;

 public:
  TargetSubParticle(double x, double y, double z) : SubParticle({x, y, z}) { };

  inline const double x() const noexcept { return p_[0]; };
  inline const double y() const noexcept { return p_[1]; };
  inline const double z() const noexcept { return p_[2]; };
  inline double& x() noexcept { return p_[0]; };
  inline double& y() noexcept { return p_[1]; };
  inline double& z() noexcept { return p_[2]; };
};

typedef std::vector<RobotSubParticle> RobotSubParticles;
typedef std::vector<TargetSubParticle> TargetSubParticles;
typedef std::vector<double> WeightSubParticles;

} // namespace pfuclt::particle

#endif //PFUCLT_SUBPARTICLE_HPP

#include "subparticle.hpp"

#include <stdexcept>


namespace pfuclt::particle {

RobotSubparticle::RobotSubparticle(double x_a, double y_a, double z_a, double yaw_a)
  : Subparticle(), x(x_a), y(y_a), z(z_a), yaw(yaw_a) { }

double& RobotSubparticle::operator[](int i)
{
    if      (i == 0) return x;
    else if (i == 1) return y;
    else if (i == 2) return z;
    else if (i == 3) return yaw;
    throw std::out_of_range("Index is out of range");
}

const double& RobotSubparticle::operator[](int i) const
{
    if      (i == 0) return x;
    else if (i == 1) return y;
    else if (i == 2) return z;
    else if (i == 3) return yaw;
    throw std::out_of_range("Index is out of range");
}


TargetSubparticle::TargetSubparticle(double x_a, double y_a, double z_a)
  : Subparticle(), x(x_a), y(y_a), z(z_a) { }

double& TargetSubparticle::operator[](int i)
{
    if      (i == 0) return x;
    else if (i == 1) return y;
    else if (i == 2) return z;
    throw std::out_of_range("Index is out of range");
}

const double& TargetSubparticle::operator[](int i) const
{
    if      (i == 0) return x;
    else if (i == 1) return y;
    else if (i == 2) return z;
    throw std::out_of_range("Index is out of range");
}

} // namespace pfuclt::particle
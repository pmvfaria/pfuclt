#include "map/landmark_map.hpp"

#include <stdexcept>


namespace pfuclt::map {

bool Landmark::isInside(const LandmarkMap& map) const
{
  return map.contains(x, y);
}


std::ostream& operator<<(std::ostream& os, const Landmark& p)
{
  return os << "Landmark " << p.id << " located at {x=" << p.x << ", y=" << p.y << '}';
}


LandmarkMap::LandmarkMap(const double& center_x, const double& center_y,
                         const double& size_x, const double& size_y)
  : center_x_(center_x), center_y_(center_y), auto_expandable_(false)
{
  if (size_x > 0.0 && size_y > 0.0)
    setSize(size_x, size_y);
  else
    throw std::invalid_argument("Size of landmark map must be positive.");
}


LandmarkMap& LandmarkMap::setAutoExpand(const bool& expand)
{
  auto_expandable_ = expand;
  return *this;
}


bool LandmarkMap::getAutoExpand() const
{
  return auto_expandable_;
}


LandmarkMap& LandmarkMap::setSize(const double& size_x, const double& size_y)
{
  size_x_ = size_x;
  size_y_ = size_y;
  half_x_ = size_x / 2.0;
  half_y_ = size_y / 2.0;
  limit_x_left_ = center_x_ - half_x_;
  limit_x_right_ = center_x_ + half_x_;
  limit_y_down_ = center_y_ - half_y_;
  limit_y_up_ = center_y_ + half_y_;

  return *this;
}


bool LandmarkMap::contains(const double& x, const double& y) const
{
  return x >= limit_x_left_ && x <= limit_x_right_ &&
         y >= limit_y_down_ && y <= limit_y_up_;
}


bool LandmarkMap::addLandmark(const int& id, const double& height,
                              const double& x, const double& y)
{
  if (contains(x, y)) {
    landmarks.emplace_back(Landmark{id, height, x, y});
    return true;
  }
  else {
    if (auto_expandable_) {
      // Increase size of map to contain this landmark at its edge
      setSize(2 * (x - center_x_), 2 * (y - center_y_));
      landmarks.emplace_back(Landmark{id, height, x, y});
      return true;
    }
    else
      return false;
  }
}


std::ostream& LandmarkMap::print(std::ostream& os) const
{
  os << "Landmark map with center {x="  << center_x_ << ", y=" << center_y_ <<
     "}, size {x=" << size_x_ << ", y=" << size_y_ << "}, and landmarks:" << std::endl;

  for (const auto& l : landmarks)
    os << '\t' << l << std::endl;

  return os;
}


std::ostream& operator<<(std::ostream& os, const LandmarkMap& map)
{
  return map.print(os);
}

} // namespace pfuclt::map

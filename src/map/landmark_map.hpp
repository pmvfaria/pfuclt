//
// Created by glawless on 8/8/18.
//

#ifndef PFUCLT_LANDMARK_MAP_HPP
#define PFUCLT_LANDMARK_MAP_HPP

#include <vector>
#include <ostream>

namespace pfuclt::map {

class LandmarkMap;

struct Landmark {
  const int id;
  const double x, y;

  bool isInside(const LandmarkMap& map) const;
};
std::ostream& operator<<(std::ostream &os, const Landmark &p);

class LandmarkMap {
 private:
  const double center_x_, center_y_;
  double half_x_, half_y_;
  double size_x_, size_y_;
  double limit_x_left_, limit_x_right_, limit_y_up_, limit_y_down_;

 public:
  bool auto_expandable{false};
  std::vector<Landmark> landmarks;

  LandmarkMap() = delete;
  /**
   * @brief Create a new square map with landmarks
   * @param center_x the x coordinate of the origin of the map, in meters
   * @param center_y the y coordinate of the origin of the map, in meters
   * @param size_x the x coordinate of the total size of the map, in meters
   * @param size_y the y coordinate of the total size of the map, in meters
   */
  LandmarkMap(const double& center_x, const double& center_y, const double& size_x, const double& size_y);

  /**
   * @brief Sets the auto-expand flag
   * @details An auto-expandable map will increase its size as needed with new landmarks added
   * @param expand boolean flag
   */
  LandmarkMap& setAutoExpand(const bool &expand);

  /**
   * @brief Set a new size to the map
   * @param size_x new size in meters, x coordinate
   * @param size_y new size in meters, y coordinate
   */
  LandmarkMap& setSize(const double &size_x, const double &size_y);

  /**
   * @brief Check if a point is within the map
   * @remark Point should be in the same frame as the map
   * @param x x coordinate of the point, in meters
   * @param y y coordinate of the point, in meters
   * @return Whether the point is within the map
   */
  bool contains(const double& x, const double& y) const;

  /**
   * @brief add a landmark at specified coordinates
   * @remark The coordinates should be in the same frame as the map
   * @param id index of the landmark
   * @param x x coordinate, in meters
   * @param y y coordinate, in meters
   * @return Whether the landmark could be added or not (e.g. if outside of map and not auto-expandable)
   */
  bool addLandmark(const int& id, const double& x, const double& y);

  std::ostream& print(std::ostream& os) const;
};

std::ostream& operator<<(std::ostream &os, const LandmarkMap& map);

} // namespace pfuclt::map

#endif //PFUCLT_LANDMARK_MAP_HPP

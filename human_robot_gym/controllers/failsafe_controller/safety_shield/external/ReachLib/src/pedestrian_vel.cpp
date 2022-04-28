/*
This file is part of Reach-RI.

Reach-RI is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
TUM, either version 3 of the License, or
(at your option) any later version.

Reach-RI is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with Foobar.  If not, see https://www.gnu.org/licenses/.
*/

#include <string>
#include <vector>

#include "cylinder_perimeter.hpp"
#include "pedestrian.hpp"
#include "pedestrian_vel.hpp"
#include "point.hpp"
#include "obstacle.hpp"
#include "system.hpp"


namespace obstacles {
namespace pedestrian {
namespace vel {

PedestrianVel::PedestrianVel(const System& sensor, double height,
                             double arm_span, double max_v, double offset,
                             const Point& init_pos) :
                             Pedestrian(sensor, height, arm_span, offset, init_pos),
                             max_v_(max_v) {
  //  NO TODO
}

const bool PedestrianVel::intersection(std::vector<Point> targets) {
  bool intersection = false;
  for (const auto& it : targets) {
    intersection = this->occupancy_[0].intersection(targets);
  }
  return intersection;
}

std::vector<CylinderPerimeter> PedestrianVel::update(double t_a, double t_b,
                                                     std::vector<Point> p,
                                                     std::vector<Point> v) {
  // Get the position of the cylinder using the equations of motion: x = x0 + max_v*t
  double vel_rad = 0.5*this->arm_span_ + this->system.measurement_error_pos_ +
                   (this->max_v_ + this->system.measurement_error_vel_) * (t_b + this->system.delay_);

  this->occupancy_ = {CylinderPerimeter(Point(p[0].x, p[0].y, this->offset_), this->height_, vel_rad)};
  return this->occupancy_;
}



}  // namespace vel
}  // namespace pedestrian
}  // namespace obstacles

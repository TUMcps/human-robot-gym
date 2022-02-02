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

#include <vector>

#include "occupancy_container.hpp"
#include "point.hpp"
#include "sphere.hpp"

namespace occupancy_containers {
namespace sphere {

Sphere::Sphere(Point p, double r) : p_(p), r_(r) {
  //  NO TODO
}

bool Sphere::intersection(const std::vector<Point>& targets) {
  double intersection = false;
  for (const auto& it : targets) {
    if (Point::norm(it, this->p_) < 0) {
      intersection = true;
    }
  }
  return intersection;
}
}  // namespace sphere
}  // namespace occupancy_containers

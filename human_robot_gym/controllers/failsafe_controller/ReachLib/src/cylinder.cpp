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

#include "cylinder.hpp"
#include "occupancy_container.hpp"
#include "point.hpp"

namespace occupancy_containers {
namespace cylinder {

Cylinder::Cylinder(const Point& p1, const Point& p2, double r) :
                   p1_(p1), p2_(p2), r_(r) {
  //  NO TODO
}

const bool Cylinder::intersection(const std::vector<Point>& targets) {
  bool intersect = false;
  for (const auto& p : targets) {
    // The cylinder is defined by a line segment from
    // the center of the top circle to the center of the bottom circle.
    Point d = this->p2_ - this->p1_;
    Point p1d = p - this->p1_;
    Point p2d = p - this->p2_;
    double h = Point::norm(d);
    double hsq = pow(h, 2);
    double dotprod = Point::inner_dot(p1d, d);

    // Check whether the point lies higher or lower than the line segment
    // in the cylinders coordinates
    // If the point lies outside the cylinders domain given an infinite radius
    // (point lies higher or lower than the cylinders top/bottom):
    if ((dotprod < 0.0) || (dotprod > hsq)) {
      return false;
    // If the point lies next to the line segment in the cylinders coordinates:
    // check if the shortes point to line distance exceeds the cylinders radius
    // if yes: no intersection
    } else {
      double rsq = pow(this->r_, 2);
      double dsq = Point::inner_dot(p1d, p1d) - ((dotprod*dotprod)/hsq);
      if (dsq > rsq) {
          return false;
      } else {
          intersect = true;
      }
    }
  }
  return intersect;
}

}  //  namespace cylinder
}  //  namespace occupancy_containers

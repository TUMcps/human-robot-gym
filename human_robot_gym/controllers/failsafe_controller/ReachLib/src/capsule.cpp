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

#include <tuple>
#include <vector>

#include "capsule.hpp"
#include "cylinder.hpp"
#include "occupancy_container.hpp"
#include "point.hpp"
#include "sphere.hpp"

namespace occupancy_containers {
namespace capsule {

Capsule::Capsule(const Point& p1, const Point& p2, double r) :
                 p1_(p1), p2_(p2), r_(r),
                 OccupancyContainer() {
  //  NO TODO
}

Capsule::Capsule(const sphere::Sphere& s) :
                 p1_(s.p_), p2_(s.p_), r_(s.r_),
                 OccupancyContainer() {
  //  NO TODO
}

Capsule::Capsule(const cylinder::Cylinder& c) :
                 p1_(c.p1_), p2_(c.p1_), r_(c.r_),
                 OccupancyContainer() {
  //  NO TODO
}

Capsule::Capsule(const OccupancyContainer& o) :
                 p1_(Point()), p2_(Point()), r_(0.0),
                 OccupancyContainer() {
  //  NO TODO
}

Capsule Capsule::minkowski(const Capsule& c1, const Capsule& c2) {
  Point p = c1.p1_ + c2.p1_;
  return Capsule(p, p, c1.r_ + c2.r_);
}

Capsule Capsule::capsuleEnclosure(const Capsule& c1, const Capsule& c2) {
    double ri = 0.0;
    double rj = 0.0;
    Point pi = Point();
    Point pj = Point();

    if (c1.r_ >= c2.r_) {
        ri = c1.r_;
        rj = c2.r_;
        pi = c1.p1_;
        pj = c2.p1_;
    } else {
        ri = c2.r_;
        rj = c1.r_;
        pi = c2.p1_;
        pj = c1.p1_;
    }

    Point x = pi - pj;
    std::tuple<double, double> ab = Capsule::alphaBeta(ri, rj, x);
    Point pk = Capsule::pk(pj, x, std::get<1>(ab));
    double dis = Point::norm(pi, pk);

    return Capsule(c1.p1_, c2.p1_, ri);
}

Capsule Capsule::ballEnclosure(const Capsule& c1, const Capsule& c2) {
  double ri = 0.0;
  double rj = 0.0;
  Point pi;
  Point pj;

  // Determine which capsule has the greater radius
  if (c1.r_ >= c2.r_) {
    ri = c1.r_;
    rj = c2.r_;
    pi = c1.p1_;
    pj = c2.p1_;
  } else {
    ri = c2.r_;
    rj = c1.r_;
    pi = c2.p1_;
    pj = c1.p1_;
  }

  Point x = pi - pj;
  std::tuple<double, double> ab = Capsule::alphaBeta(ri, rj, x);
  Point pk = Capsule::pk(pj, x, std::get<1>(ab));
  pi.x = (pi.x + pk.x)/2.0;
  pi.y = (pi.y + pk.y)/2.0;
  pi.z = (pi.z + pk.z)/2.0;

  return Capsule(pi, pi, (ri + rj + std::get<0>(ab))/2.0);
}

Point Capsule::pk(Point pj, Point x, double b) {
  double norm = Point::norm(x);
  if (norm == 0.0) {
    return pj;
  }
  Point pk = Point();
  pk.x = pj.x + (x.x/norm)*b;
  pk.y = pj.y + (x.y/norm)*b;
  pk.z = pj.z + (x.z/norm)*b;
  return pk;
}

std::tuple<double, double> Capsule::alphaBeta(double ri, double rj, Point x) {
    double n = Point::norm(x);
    return std::make_tuple(std::max(ri - rj, n), std::min(ri - rj, n));
}

bool Capsule::intersection(const std::vector<Point>& targets) {
  bool intersect = false;
  for (auto& it : targets) {
    intersect = point_capsule_intersection(*this, targets);
    if (intersect == true) {
      return true;
    }
  }
  return intersect;
}

bool Capsule::point_capsule_intersection(const Capsule& c, const std::vector<Point>& points, double r) {
  // Check whether the capsule is a ball
  if (c.p1_ == c.p2_) {
    for (const auto& p : points) {
      return 0 <= c.r_ + r - Point::norm(c.p1_, p);
    }
  }
  for (const auto& p : points) {
    // The capsules cylinder is defined by a line segment from
    // the center of the top circle to the center of the bottom circle.
    Point d = c.p2_ - c.p1_;
    Point p1d = p - c.p1_;
    Point p2d = p - c.p2_;
    double h = Point::norm(d);
    double hsq = pow(h, 2);
    double dotprod = Point::inner_dot(p1d, d);

    // Check whether the point lies higher or lower than the line segment
    // in the cylinders coordinates
    // If the point lies outside the cylinders domain given an infinite radius
    // (point lies higher or lower than the cylinders top/bottom):
    // Check for intersections with the capsules halfspheres (the same as a sphere radius check)
    if ((dotprod < 0.0) || (dotprod > hsq)) {
      if (Point::norm(p1d) > c.r_ + r && Point::norm(p2d) > c.r_ + r) {
          return false;
      } else {
          return true;
      }
    // If the point lies next to the line segment in the cylinders coordinates:
    // check if the shortes point to line distance exceeds the cylinders radius
    // if yes: no intersection
    } else {
      double rsq = pow(c.r_ + r, 2);
      double dsq = Point::inner_dot(p1d, p1d) - ((dotprod*dotprod)/hsq);
      if (dsq > rsq) {
          return false;
      } else {
          return true;
      }
    }
  }
}

}  //  namespace capsule
}  //  namespace occupancy_containers

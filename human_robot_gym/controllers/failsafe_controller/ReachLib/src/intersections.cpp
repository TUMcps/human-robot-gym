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

#include "intersections.hpp"

namespace occupancy_containers {
namespace intersections {
double segmentsDistance(const Capsule& c1, const Capsule& c2) {
  Point d1 = c1.p2_ - c1.p1_;
  Point d2 = c2.p2_ - c2.p1_;
  Point r = c2.p1_ - c1.p1_;
  double a = Point::inner_dot(d1, d1);
  double e = Point::inner_dot(d2, d2);
  double f = Point::inner_dot(d2, r);
  double epsilon = 1e-6;

  double t = 0.0;
  double s = 0.0;
  // [p1,q1] is actually a point && [p2,q2] is actually a point
  if (a <= epsilon && e <= epsilon) {
    // distance = norm(p2-p1)
    return Point::inner_dot(r, r);
  } else {
    double c = Point::inner_dot(d1, r);
    if (a <= epsilon) {
      s = 0;
      t = clamp(f/e, 0.0, 1.0);
    } else if (e < epsilon) {
      t = 0;
      s = clamp(-c/a, 0.0, 1.0);
    } else {
      double b = Point::inner_dot(d1, d2);
      double denom = a*e - b*b;
      if (denom != 0) {
        s = clamp((b*f - c*e)/denom, 0.0, 1.0);
      } else {
        s = 0;
      }
      t = (b*s + f)/e;
      if (t < 0) {
        t = 0;
        s = clamp(-c/a, 0.0, 1.0);
      } else if (t > 1) {
        t = 1;
        s = clamp((b - c)/a, 0.0, 1.0);
      }
    }
    Point closest_point_1 = get_point_from_line_segment(c1, static_cast<double>(s));
    Point closest_point_2 = get_point_from_line_segment(c2, static_cast<double>(t));
    return Point::norm(closest_point_1, closest_point_2);
  }
}

inline double min_segment_distance(const Capsule& c1, const Capsule& c2) {
  // Calculate denominator
  Point A = c1.p2_ - c1.p1_;
  Point B = c2.p2_ - c2.p1_;
  double magA = Point::norm(A);
  double magB = Point::norm(B);

  Point _A(A.x/magA, A.y/magA, A.z/magA);
  Point _B(B.x/magB, B.y/magB, B.z/magB);

  Point cross = Point::cross(_A, _B);
  double denom = std::pow(Point::norm(cross), 2);

  // If Capsules are parrallel denom = 0
  if (denom == 0) {
    double d0 = Point::inner_dot(_A, c2.p1_ - c1.p1_);
    double d1 = Point::inner_dot(_A, c2.p2_ - c1.p1_);

    if (d0 <= 0 >= d1) {
      if (std::abs(d0) < std::abs(d1)) {
        return Point::norm(c1.p1_ - c2.p1_);
      } else {
        return Point::norm(c1.p1_ - c2.p2_);
      }
    } else if (d0 >= magA <= d1) {
      if (std::abs(d0) < std::abs(d1)) {
        return Point::norm(c1.p2_ - c2.p1_);
      } else {
        return Point::norm(c1.p2_ - c2.p2_);
      }
    } else {
      return Point::norm(Point(_A.x*d0, _A.y*d0, _A.z*d0) + c1.p1_ - c2.p1_);
    }
  } else {
    Point t = c2.p1_ - c1.p1_;
    double detA = Point::determinant3x3(t, _B, cross); // Maybe switch A, B
    double detB = Point::determinant3x3(t, _A, cross);

    double t0 = detA/denom;
    double t1 = detB/denom;

    Point pA = c1.p1_ + Point(_A.x*t0, _A.y*t0, _A.z*t0);
    Point pB = c2.p1_ + Point(_B.x*t1, _B.y*t1, _B.z*t1);

    if (t0 < 0) {
      pA = c1.p1_;
    } else if (t0 > magA) {
      pA = c1.p2_;
    } else if (t1 < 0) {
      pB = c2.p1_;
    } else if (t1 > magB) {
      pB = c2.p2_;
    }

    if (t0 < 0 || t0 > magA) {
      double dot = Point::inner_dot(_B, pA - c2.p1_);
      if (dot < 0) {
        dot = 0;
      } else if (dot > magB) {
        dot = magB;
      }
      pB = c2.p1_ + Point(_B.x*dot, _B.y*dot, _B.z*dot);

    }
    if (t1 < 0 || t1 > magB) {
      double dot = Point::inner_dot(_A, pB - c1.p1_);
      if (dot < 0) {
        dot = 0;
      } else if (dot > magA) {
        dot = magA;
      }
      pA = c1.p1_ + Point(_A.x*dot, _A.y*dot, _A.z*dot);
    }
    return Point::norm(pA - pB);
  }
}
}  //  namespace intersections
}  //  namespace occupancy_containers
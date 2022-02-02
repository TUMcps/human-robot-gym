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

#include <cassert>
#include <string>
#include <vector>

#include "body_part.hpp"
#include "body_part_accel.hpp"
#include "capsule.hpp"
#include "occupancy.hpp"
#include "point.hpp"



namespace occupancies {
namespace body_parts {
namespace accel {

BodyPartAccel::BodyPartAccel(std::string name, double thickness, double max_a1, double max_a2) :
                             BodyPart(name, thickness),
                             max_a1_(max_a1), max_a2_(max_a2) {
  //  NO TODO
}

void BodyPartAccel::update(const std::vector<Point>& p,
                           const std::vector<Point>& v,
                           double t_a, double t_b,
                           double measurement_error_pos,
                           double measurement_error_vel,
                           double delay) {
  // The vectors must contain exactly 2 points
  assert(("Vector p must have exactly two entries for BodyPartAccel", size(p) == 2));
  assert(("Vector v must have exactly two entries for BodyPartAccel", size(v) == 2));
  // translate vector entries to proximal and distal joint values
  Point p1 = p[0];
  Point p2 = p[1];
  Point v1 = v[0];
  Point v2 = v[1];

  // Check whether this capsule is a ball
  bool b;
  if (p1 == p2) {
      b = true;
  } else {
      b = false;
  }

  // Calculate the occupancies of the proximal and distal joint up to t_a
  Capsule rp1_t1 = BodyPartAccel::ry(p1, v1, 1, t_a, delay,
                                     measurement_error_pos, measurement_error_vel);
  Capsule rp1_t2 = BodyPartAccel::ry(p1, v1, 1, t_b, delay,
                                     measurement_error_pos, measurement_error_vel);  // was p2
  Capsule b1 = Capsule::ballEnclosure(rp1_t1, rp1_t2);

  if (b) {
    // Add the thickness
    rp1_t2.r_ += this->thickness_;

    this->occupancy_ = rp1_t2;  // was rp1_t1
  } else {
    Capsule rp2_t1 = BodyPartAccel::ry(p2, v2, 2, t_a, delay,
                                     measurement_error_pos, measurement_error_vel);  // was p1
    Capsule rp2_t2 = BodyPartAccel::ry(p2, v2, 2, t_b, delay,
                                     measurement_error_pos, measurement_error_vel);
    Capsule b2 = Capsule::ballEnclosure(rp2_t1, rp2_t2);

    // Add the thickness to the ball with the larger radius (determines capsule radius)
    if (b1.r_ >= b2.r_) {
        b1.r_ += this->thickness_;
    } else {
        b2.r_ += this->thickness_;
    }
    this->occupancy_ = Capsule::capsuleEnclosure(b1, b2);
  }
}

bool BodyPartAccel::intersection(std::vector<Point> targets) {
  return this->occupancy_.intersection(targets);
}

Capsule BodyPartAccel::ry(const Point& p, const Point& v,
                          int index, double t_b, double delay,
                          double measurement_error_pos,
                          double measurement_error_vel) {
  Point y;
  y = p;
  Point dy = v;

  double max_a = 0.0;
  if (index == 1) {
    max_a = this->max_a1_;
  } else {
    max_a = this->max_a2_;
  }

  dy.x *= t_b;
  dy.y *= t_b;
  dy.z *= t_b;

  Point zero = Point();
  // Calculate the bound for the integral of y(t) = y(0) + dy(t)*t + 0.5*ddy(0)*t^2
  Capsule Bydy = Capsule::minkowski(Capsule(y, y, measurement_error_pos),
                 Capsule(dy, dy, measurement_error_vel * (t_b + delay)));
  return Capsule::minkowski(Bydy, Capsule(zero, zero, max_a / 2.0 * pow(t_b + delay, 2)));
}
}  // namespace accel
}  // namespace body_parts
}  // namespace occupancies

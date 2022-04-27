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
#include "body_part_vel.hpp"
#include "capsule.hpp"
#include "occupancy.hpp"
#include "point.hpp"



namespace occupancies {
namespace body_parts {
namespace vel {

BodyPartVel::BodyPartVel(std::string name, double thickness,
                         double max_v1, double max_v2) :
                         BodyPart(name, thickness),
                         max_v1_(max_v1), max_v2_(max_v2) {
  //  NO TODO
}

const bool BodyPartVel::intersection(std::vector<Point> targets) {
  return this->occupancy_.intersection(targets);
}

void BodyPartVel::update(const std::vector<Point>& p,
                         const std::vector<Point>& v,
                         double t_a, double t_b,
                         double measurement_error_pos,
                         double measurement_error_vel,
                         double delay) {
  // The vectors must contain exactly 2 points
  assert(("Vector p must have exactly two entries for BodyPartAccel", size(p) == 2));

  // translate vector entries to proximal and distal joint values
  Point p1 = p[0];
  Point p2 = p[1];

  // Check whether this capsule is a ball
  bool b;
  if (p1 == p2) {
      b = true;
  } else {
      b = false;
  }

  // Calculate the occupancies of the proximal and distal joint up to t_a
  Capsule rp1_t1 = BodyPartVel::ry(p1, 1, t_a, delay,
                                     measurement_error_pos);
  Capsule rp1_t2 = BodyPartVel::ry(p1, 1, t_b, delay,
                                     measurement_error_pos);  // was p2
  Capsule b1 = Capsule::ballEnclosure(rp1_t1, rp1_t2);

  if (b) {
    // Add the thickness
    rp1_t2.r_ += this->thickness_;

    this->occupancy_ = rp1_t2;  // was rp1_t1
  } else {
    Capsule rp2_t1 = BodyPartVel::ry(p2, 2, t_a, delay,
                                     measurement_error_pos);  // was p1
    Capsule rp2_t2 = BodyPartVel::ry(p2, 2, t_b, delay,
                                     measurement_error_pos);
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

Capsule BodyPartVel::ry(const Point& p, int index, double t_b,
           double delay, double measurement_error_pos) {
  double max_v;
  if (index == 1) {
    max_v = this->max_v1_;
  } else {
    max_v = this->max_v2_;
  }

  // RP
  return Capsule(p, p, measurement_error_pos + max_v * t_b + delay);
}

}  // namespace vel
}  // namespace body_parts
}  // namespace occupancies

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

#include "capsule.hpp"
#include "cylinder.hpp"
#include "occupancy.hpp"
#include "occupancy_container.hpp"
#include "point.hpp"
#include "sphere.hpp"

#ifndef REACH_LIB_INCLUDE_INTERSECTIONS_HPP_
#define REACH_LIB_INCLUDE_INTERSECTIONS_HPP_

namespace occupancy_containers {
//! \namespace Defines intersection functions among all occupancy_containers.
//! If a new occupancy_container type is defined, intersection functions with
//! among its own type, Point-type, and any other occupancy_container type must be defined.
namespace intersections {

//! \typedef Defines a shortcut to the Cylinder class
typedef cylinder::Cylinder Cylinder;

//! \typedef Defines a shortcut to the Capsule class
typedef capsule::Capsule Capsule;

//! \typedef Defines a shortcut to the Sphere class
typedef sphere::Sphere Sphere;

//! \brief Limits the value of a floating point variable by applying
//! upper and lower bounds.
//! \param[in] value Unclamped value.
//! \param[in] lower Lower value bound.
//! \param[in] upper Upper value bound.
inline double clamp(double value, double lower = 0.0, double upper = 1.0) {
  if (value > upper) {
    return upper;
  } else if (value < lower) {
    return lower;
  } else {
    return value;
  }
}

//! \brief Calculates the shortest distance between a Cartesian point (x, y, z)
//! and a finite line segents.
//! \param[in] p A Point object
//! \param[in] c A Capsule object defining a line segment
inline double point_line_segment_dist(const Point& p, const Capsule& c) {
  // https://de.mathworks.com/matlabcentral/answers/260593-distance-between-points-and-a-line-segment-in-3d
  // Vector from start to end of segment
  Point se = c.p2_ - c.p1_;
  // Length of segment
  double dse = Point::norm(se);
  // Distance from start point to point
  Point sp = p - c.p1_;
  double dsp = Point::norm(sp);
  // Distance from end point to point
  Point ep = p - c.p2_;
  double dep = Point::norm(ep);
  // Type 1: Point closest to end point
  if (sqrt(pow(dse, 2) + pow(dep, 2)) <= dsp) {
    return dep;
  }
  // Type 2: Point closest to start point
  if (sqrt(pow(dse, 2) + pow(dsp, 2)) <= dep) {
    return dsp;
  }
  // Type 3: Point in between start and end point
  return (Point::norm(Point::cross(se, ep))/dse);
}

inline Point get_point_from_line_segment(const Capsule& c, double t) {
  return Point(c.p1_.x + (c.p2_.x - c.p1_.x)*t,
               c.p1_.y + (c.p2_.y - c.p1_.y)*t,
               c.p1_.z + (c.p2_.z - c.p1_.z)*t);
}

//! \brief Calculates the shortest distance between two finite line segents.
//! \param[in] c1 A Capsule object defining a line segment
//! \param[in] c2 A Capsule object defining a line segment
double segmentsDistance(const Capsule& c1, const Capsule& c2);

double min_segment_distance(const Capsule& c1, const Capsule& c2);

//! \brief Calculates whether the shortest distance between two capsules
//! \param[in] c1 An object of type Capsule
//! \param[in] c2 An object of type Capsule
inline double capsule_capsule_dist(const Capsule& c1, const Capsule& c2) {
  // First check if any capsule is a sphere
  bool is_sphere1 = c1.p1_ == c1.p2_;
  bool is_sphere2 = c2.p1_ == c2.p2_;
  // Sphere collision check
  if (is_sphere1 && is_sphere2) {
    return Point::norm(c1.p1_, c2.p1_) - c1.r_ - c2.r_;
  // Sphere and capsule collision check
  } else if (is_sphere1 && !is_sphere2) {
    return point_line_segment_dist(c1.p1_, c2) - c1.r_ - c2.r_;
  // Sphere and capsule collision check
  } else if (!is_sphere1 && is_sphere2) {
    return point_line_segment_dist(c2.p1_, c1) - c1.r_ - c2.r_;
  // Capsule and capsule collision
  } else {
    // return segmentsDistance(c1, c2) - c1.r_ - c2.r_;
    // return c_c_distance(c1, c2);
    return min_segment_distance(c1, c2) - c1.r_ - c2.r_;
  }
}

//! \brief Determines whether there exists an intersection between two capsules
//! \param[in] c1 An object of type Capsule
//! \param[in] c2 An object of type Capsule
inline bool capsule_capsule_intersection(const Capsule& c1, const Capsule& c2) {
  return capsule_capsule_dist(c1, c2) < 0.0;
}

//! \brief Calculates the closest distance between two VERTICALLY extruded cylinders.
//! \param[in] c1 An object of type Cylinder
//! \param[in] c2 An object of type Cylinder
inline double cylinder_cylinder_dist(const Cylinder& c1, const Cylinder& c2) {
  return c1.r_ + c2.r_ - Point::norm(c1.p1_ - c2.p1_);
}

//! \brief Determines whether there exists an intersection
//! between two VERTICALLY extruded cylinders.
//! \param[in] c1 An object of type Cylinder
//! \param[in] c2 An object of type Cylinder
inline bool cylinder_cylinder_intersection(const Cylinder& c1, const Cylinder& c2) {
  return cylinder_cylinder_dist(c1, c2) < 0.0;
}

//! \brief Calculates the closest distance between an object of type cylinder
//! and an object of type capsule where cylinders are approximated as capsules
//! \param[in] cy An object of type Cylinder
//! \param[in] ca An object of type Capsule
inline double cylinder_capsule_dist(const Cylinder& cy, const Capsule& ca) {
  Capsule cyc = Capsule(cy.p1_, cy.p2_, cy.r_);
  return capsule_capsule_dist(cyc, ca);
}

//! \brief Determines whether an object of type Cylinder
//! and an object of type Capsule intersect.
//! \param[in] cy An object of type Cylinder
//! \param[in] ca An object of type Capsule
inline bool cylinder_capsule_intersection(const Cylinder& cy, const Capsule& ca) {
  return cylinder_capsule_dist(cy, ca) < 0.0;
}

//! \brief Calculates the closest distance between an object of type Sphere
//! and an object of type Capsule.
//! \param[in] cy An object of type Cylinder
//! \param[in] ca An object of type Capsule
inline double sphere_capsule_dist(const Sphere& s, const Capsule& c) {
  return capsule_capsule_dist(static_cast<Capsule>(s), c);
}

//! \brief Determines whether an object of type Sphere
//! and an object of type Capsule intersect.
//! \param[in] cy An object of type Cylinder.
//! \param[in] ca An object of type Capsule.
inline double sphere_capsule_intersection(const Sphere& s, const Capsule& c) {
  return capsule_capsule_intersection(static_cast<Capsule>(s), c);
}

//! \brief Calculates the closest distance between two objects of type Sphere.
//! \param[in] s1 An object of type Sphere.
//! \param[in] s2 An object of type Sphere.
inline double sphere_sphere_dist(const Sphere& s1, const Sphere& s2) {
  return s1.r_ + s2.r_ - Point::norm(s1.p_, s2.p_);
}

//! \brief Determines whether two objects of type Sphere intersect.
//! \param[in] s1 An object of type Sphere.
//! \param[in] s2 An object of type Sphere.
inline double sphere_sphere_intersection(const Sphere& s1, const Sphere& s2) {
  return sphere_sphere_dist(s1, s2) < 0.0;
}

}  //  namespace intersections
}  //  namespace occupancy_containers
#endif  //  REACH_LIB_INCLUDE_INTERSECTIONS_HPP_

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

#ifndef REACH_LIB_INCLUDE_CYLINDER_HPP_
#define REACH_LIB_INCLUDE_CYLINDER_HPP_

namespace occupancy_containers {
namespace cylinder {
//! This class defines Cylinder based occupancies
//! used in the pedestrian model defiend in:
//! http://mediatum.ub.tum.de/doc/1379662/905746.pdf
//! and static velocity based perimeters
class Cylinder : public  OccupancyContainer {
 public:
  //! \brief Radius of the cylinder
  double r_ = 0.0;

  //! \brief Center of the cylinders initial top circle
  Point p1_ = Point();

  //! \brief Center of the cylinders initial bottom circle
  Point p2_ = Point();

  //! \brief Empty constructor
  Cylinder() {}

  //! \brief Instantiates a cylinder occupancy
  //! \param[in] p A list of two points serving as
  //!              top and bottom indicators for the cylinder
  //! \param[in] r Radius of the cylinder
  Cylinder(const Point& p1, const Point& p2, double r);

  //! \brief Empty destructor
  ~Cylinder() {}

  //! \brief Determines whether there exists an intersection between
  //!        a cylinder and any point within targets.
  //! \param[in] targets List of points of interes in Cartesian (x y z)
  const bool intersection(const std::vector<Point>& targets);

  //! \brief Determines whether this cylinder and a capsule c intersect.
  //! \param[in] c A Capsule object checked against this cylinder
  //! -> We can't allow circular inclusion
  // bool intersection(const Capsule& c);
};
}  //  namespace cylinder
}  //  namespace occupancy_containers
#endif  //  REACH_LIB_INCLUDE_CYLINDER_HPP_

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

#ifndef REACH_LIB_INCLUDE_SPHERE_HPP_
#define REACH_LIB_INCLUDE_SPHERE_HPP_

namespace occupancy_containers {
namespace sphere {

class Sphere : public OccupancyContainer {
 public:
  //! \brief Origin of the sphere
  Point p_ = Point();

  //! \brief Radius of the sphere
  double r_ = 0.0;

  //! \brief Empty constructor
  Sphere() {}

  //! \brief Instatiates an object of type Sphere
  Sphere(Point p, double r);

  //! \brief Empty destructor
  ~Sphere() {}

  //! \brief Determines whether there exists an intersection between
  //!        this Sphere and any point within targets.
  //! \param[in] targets List of points of interes in Cartesian (x y z)
  bool intersection(const std::vector<Point>& targets);
};
}  // namespace sphere
}  // namespace occupancy_containers
#endif  //  REACH_LIB_INCLUDE_SPHERE_HPP_

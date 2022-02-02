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

#include "point.hpp"

#ifndef REACH_LIB_INCLUDE_OCCUPANCY_CONTAINER_HPP_
#define REACH_LIB_INCLUDE_OCCUPANCY_CONTAINER_HPP_

namespace occupancy_containers {
//! \typedef Point describes the Point class within the point namespace
typedef point::Point Point;

//! This class describes all types of media that
//! can contain reachable occupancies.
//! They contain subsets of the overall reachable
//! occupancy and are used to build OccupancyModels.
class OccupancyContainer {
 public:
  //! \brief Empty constructor
  OccupancyContainer() {}

  //! \brief Empty destructor
  virtual ~OccupancyContainer() {}

  //! \brief Determines whether there exists an intersection between
  //!        an occupancy container and any point within targets.
  //!        Throws an exception if the type of OccupancyContainer
  //!        can not be determined.
  //! \param[in] targets List of points of interes in Cartesian (x y z)
  bool intersection(const std::vector<Point>& targets) {
    throw "Function occupancy_containers::OccupancyContainer intersection is not defined!";
  }
};
}  //  namespace occupancy_containers
#endif  // REACH_LIB_INCLUDE_OCCUPANCY_CONTAINER_HPP_

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

#include "occupancy.hpp"
#include "point.hpp"
#include "system.hpp"

#ifndef REACH_LIB_INCLUDE_SAFETY_PERIMETER_HPP_
#define REACH_LIB_INCLUDE_SAFETY_PERIMETER_HPP_


namespace obstacles {

//! \typedef A shortcut to the Point class
typedef point::Point Point;

//! \typedef A shortcut to the System class
typedef systems::System System;

//! This class serves as a blueprint for all
//! safety perimeter definitions.
//! This presently includes Articulated and Pedestrian models.
class Obstacle {
 public:
  //! \brief Empty constructor
  Obstacle() {}

  //! \brief Initializes the sensor attribute containing
  //!        system parameters.
  explicit Obstacle(System sensor) : system(sensor) {}

  //! \brief Empty destructor
  ~Obstacle() {}

  //! \brief Returns true if the current occupancy intersects with
  //!        any given point in 'targets'
  //! \param[in] targets A list of points in global Cartesian coordinates (x, y, z)
  //!                    checked against the current occupancy
  const bool intersection(std::vector<Point> targets) {
    throw "Function safety_perimeters::Obstacle::intersection is not defined!";
  }

  //! \brief Returns Pointers to the current occupancies of the respective model
  inline std::vector<occupancies::Occupancy*> get_occupancy_p() {
    return this->occupancy_p;
  }

  //! \brief Returns the system parameters that the model was initialized with
  inline System get_system() {
    return this->system;
  }

 protected:
  //! \brief Containins system information (measurement errors/delay)
  System system = System();

  //! \brief Contains pointers to the most recent state of all occupancy segments
  std::vector<occupancies::Occupancy*> occupancy_p = {};
};
}  //  namespace obstacles
#endif  //  REACH_LIB_INCLUDE_SAFETY_PERIMETER_HPP_

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

#include <iostream>
#include <string>
#include <vector>

#include "occupancy_container.hpp"
#include "point.hpp"

#ifndef REACH_LIB_INCLUDE_OCCUPANCY_MODEL_HPP_
#define REACH_LIB_INCLUDE_OCCUPANCY_MODEL_HPP_


namespace occupancies {
//! \typedef A shortcut to the Point type
typedef point::Point Point;

//! \typedef A shortcut to the OccupancyContainer type
typedef occupancy_containers::OccupancyContainer OccupancyContainer;

//! This class defines the structure of all accepted
//! models of occupancy. This includes a container for
//! the current occupancy as well as an intersection check.
//! Occupancy can be defined using two models:
//! 1. BodySegments: A part of the human body [body part, extremity]
//! 2. SafetyField:  A cylinder encompassing reachable space of a pedestrian
class Occupancy {
 public:
  //! \brief Empty costructor
  Occupancy() {}

  //! \brief Initializes a named object of underterined type among Occupancys
  explicit Occupancy(std::string name);

  //! \brief Empty destructor
  virtual ~Occupancy() {}

  //! \brief Returns true if the current occupancy intersects with
  //!        any given point in 'targets'
  //! \param[in] targets A list of points in global Cartesian coordinates (x, y, z)
  //!                    checked against the current occupancy
  const bool intersection(std::vector<point::Point> targets) {
    throw "Function occupancy_models::Occupancy::intersection is not defined!";
  }

  //! \brief Calcualtes the current occupancy using any model
  //! \param[in] p Current joint position in Cartesian coordinartes (x, y ,z)
  //! \param[in] v Current joint velocity
  //! \param[in] t_a Start of the interval of analysis
  //! \param[in] t_b End of the interval of analysis
  //! \param[in] measurement_error_pos Measurement errors in cartesian position [m]
  //! \param[in] measurement_error_vel Measurement errors in cartesian velocity [m/s]
  //! \param[in] delay Time delay within the executing system
  void update(const std::vector<Point>& p,
              const std::vector<Point>& v = {},
              double t_a = 0.0, double t_b = 0.0,
              double measurement_error_pos = 0.0,
              double measurement_error_vel = 0.0,
              double delay = 0.0) {
    throw "Function occupancy_models::Occupancy::intersection is not defined!";
  }

  //! \brief Returns the occupancy according to its occupancy model
  //! Throws a NULL-pointer exception if the occupancy pointer is not set
  OccupancyContainer* get_occupancy_p() {
    return this->occupancy_p;
  }

  //! \brief Returns the name of the body part
  std::string get_name() {
      return this->name_;
  }

 protected:
  //! \brief Points to the current occupancy (for dynamic cast purposes)
  OccupancyContainer* occupancy_p = NULL;

  //! \brief A name that defines this occupancy
  std::string name_ = "";
};
}  //  namespace occupancies
#endif  // REACH_LIB_INCLUDE_OCCUPANCY_MODEL_HPP_

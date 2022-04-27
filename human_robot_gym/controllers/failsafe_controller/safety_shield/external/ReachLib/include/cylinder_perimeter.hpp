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

#include "cylinder.hpp"
#include "occupancy.hpp"
#include "point.hpp"

#ifndef REACH_LIB_INCLUDE_CYLINDER_PERIMETER_HPP_
#define REACH_LIB_INCLUDE_CYLINDER_PERIMETER_HPP_

namespace occupancies {
namespace cylinder_perimeters {
//! \typedef Defines the Capsule datatype within this namespace
typedef occupancy_containers::cylinder::Cylinder Cylinder;


//! \brief Defines the occupancy model of the pedestrian approach
//! from http://mediatum.ub.tum.de/doc/1379662/905746.pdf.
//! Contains a Cylinder for the occupancy of the desired interval [t_start, t_end].
//! Contains a list of cylinder sub occupancies for intervals from within [0.0, t_end].
class CylinderPerimeter : public Occupancy {
 public:
  //! \brief Empty constructor
  CylinderPerimeter();

  //! \brief Instatiates a list of cylinders that
  //!        can be used to define occupancies of
  //!        the pedestrian model from:
  //!        http://mediatum.ub.tum.de/doc/1379662/905746.pdf
  //! \param[in] init_pos Initial position of the cylinder
  //! \param[in] height Estimated height of the human
  CylinderPerimeter(const Point& init_pos, double height, double radius, std::string name = "");

  //! \brief Emtpy destructor
  ~CylinderPerimeter() {}

  //! Determines if the cylinder intersects with
  //! any point from within the targest list
  //! \param[in] Points of interest for inertsections
  const bool intersection(std::vector<Point> targets);

  //! \brief Calcualtes the current occupancy using the Pedestrian model
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
    throw "Function occupancies::cylinder_perimeters::CylinderPerimeter::update is not defined!";
  }

  //! \brief Returns the current occupancy
  inline Cylinder get_occupancy() {
    return this->occupancy_;
  }

  //! \brief Returns the list of cylindrical partial occupancies
  inline std::vector<Cylinder> get_cylinder_list() {
    return this->cylinder_list_;
  }

 private:
  //! \brief Contains the current occupancy
  Cylinder occupancy_ = Cylinder();

  //! \brief The desired occupancy interval split up into smaller intervals
  //!        represented by a list of cylinders
  std::vector<Cylinder> cylinder_list_ = {};
};
}  //  namespace cylinder_perimeters
}  //  namespace occupancies
#endif  //  REACH_LIB_INCLUDE_CYLINDER_PERIMETER_HPP_

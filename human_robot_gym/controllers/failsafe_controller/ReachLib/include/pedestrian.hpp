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

#include "cylinder_perimeter.hpp"
#include "occupancy.hpp"
#include "point.hpp"
#include "obstacle.hpp"
#include "system.hpp"

#ifndef REACH_LIB_INCLUDE_PEDESTRIAN_HPP_
#define REACH_LIB_INCLUDE_PEDESTRIAN_HPP_

namespace obstacles {
namespace pedestrian {

//! \brief A shortcut to the BodyPart type
typedef occupancies::Occupancy Occupancy;

//! \typedef A shortcut to the Cylinder type
typedef occupancies::cylinder_perimeters::CylinderPerimeter CylinderPerimeter;

//! \typedef A shortcut to the occupancy container class Cylinder
typedef occupancy_containers::cylinder::Cylinder Cylinder;


//! \brief Describes a safety perimeter in form of a cylindrical occupancy
//! based on either constant maximum velocity or live velocies and constant acceleration
//! as shown in http://mediatum.ub.tum.de/doc/1379662/905746.pdf.
class Pedestrian : public Obstacle {
 public:
  //! \brief Empty constructor
  Pedestrian() {}

  //! \brief Instantites a Pedestrian object
  //! \param[in] sensor System parameters (masurement errors, delay)
  //! \param[in] height The height of the pedestrian from the soles to the crown
  //! \param[in] arm_span The arm span estimated from finger tip to finger tip
  Pedestrian(System sensor, double height, double arm_span, double offset = 0.0,
             const Point& init_pos = Point());

  //! \brief Emtpy destructor
  ~Pedestrian() {}

  //! \brief Calcualtes the current occupancy using the pedestrian model.
  //! \param[in] p Current location of points of interest in Cartesian coordinartes (x, y ,z).
  //! \param[in] v Current velocity of points of interest [optional].
  //! \param[in] t_a Start of the interval of analysis.
  //! \param[in] t_b End of the interval of analysis.
  //! \param[in] steps Number of additional suboccupancies calculated.
  CylinderPerimeter update(double t_a, double t_b,
                           std::vector<Point> p,
                           std::vector<Point> v = {}) {
    throw "Function safety_perimeters::pedestrian::Pedestrian::update is not defined!";
  }

  //! \brief Returns true if the current occupancy intersects with
  //!        any given point in 'targets'
  //! \param[in] targets A list of points in global Cartesian coordinates (x, y, z)
  //!                    checked against the current occupancy
  const bool intersection(std::vector<Point> targets) {
    throw "Function safety_perimeters::pedestrian::Pedestrian::intersection is not defined!";
  }

  //! \brief Returns the current occupancy
  std::vector<CylinderPerimeter> get_occupancy() {
    return this->occupancy_;
  }

  //! \brief Returns the pedestrians height
  double get_height() {
    return this->height_;
  }

  //! \brief Returns the offset of the ground level
  double get_offset() {
    return this->offset_;
  }

  //! \brief Returns the mode of the reachalility analysis:
  //!        Mode returned from: [ACCEL, VEL].
  std::string get_mode() {
    throw "Function safety_perimeters::pedestrian::Pedestrian::get_mode is not defined!";
  }

 protected:
  //! \brief Arm span of the tracked pedestrian.
  //!        Estimated from finger tip to finger tip.
  double arm_span_ = 0.0;

  //! \brief Height of the tracked pedestrian.
  //!        Estimated from the soles to the crown.
  double height_ = 0.0;

  //! \brief The offset of the ground level (z) based on global coordinates
  double offset_ = 0.0;

  //! \brief Contains the current occupancy
  std::vector<CylinderPerimeter> occupancy_;
};
}  //  namespace pedestrian
}  //  namespace obstacles
#endif  //  REACH_LIB_INCLUDE_PEDESTRIAN_HPP_

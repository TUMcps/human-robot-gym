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
#include "occupancy.hpp"
#include "point.hpp"

#ifndef REACH_LIB_INCLUDE_EXTREMITY_HPP_
#define REACH_LIB_INCLUDE_EXTREMITY_HPP_

namespace occupancies {
namespace extremities {

//! \typedef A shortcut to the Capsule type
typedef occupancy_containers::capsule::Capsule Capsule;


/*! This class defines the human as a construct
 *  of extremities which describe their own occupancy.
 *  An extreity is defined by a joint, its thickness, and length.
 *  extremities are used for the maximum position based approache.
 *  They describe a sphere of constant radius about their joint.
 */
class Extremity : public Occupancy {
 public:
  //! \brief Empty constructor
  Extremity();

  //! \brief Instantiates an extremity
  Extremity(std::string name, double thickness,
            double length, double max_v);

  //! \brief Empty destructor
  ~Extremity() {}


  //! \brief Calcualtes the current occupancy using the ArticulatedPos model
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
              double delay = 0.0);

  //! \brief Returns true if the current occupancy intersects with
  //!        any given point in 'targets'
  //! \param[in] targets A list of points in global Cartesian coordinates (x, y, z)
  //!                    checked against the current occupancy
  const bool intersection(std::vector<Point> targets);

  //! \brief Returns the current occupancy
  inline Capsule get_occupancy() {
    return this->occupancy_;
  }

  //! \brief Returns the thickness of the extremity
  double get_thickness() {
      return this->thickness_;
  }

  //! \brief Returns the length of the extremity
  double get_length() {
      return this->length_;
  }

 private:
  //! \brief Conains the current occupancy
  Capsule occupancy_ = Capsule();

  //! \brief Thickness of the extremity
  double thickness_ = 0.0;

  //! \brief Length of the extremity
  double length_ = 0.0;

  //! \brief Maximum assumed velocity of the extremity
  double max_v_ = 0.0;
};
}  // namespace extremities
}  //  namespace occupancies
#endif  //  REACH_LIB_INCLUDE_EXTREMITY_HPP_

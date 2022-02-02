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

#ifndef REACH_LIB_INCLUDE_BODY_PART_HPP_
#define REACH_LIB_INCLUDE_BODY_PART_HPP_

namespace occupancies {
namespace body_parts {

//! \typedef A shortcut to the Capsule type
typedef occupancy_containers::capsule::Capsule Capsule;

/*! This class defines the human as a construct
 *  of body parts which describe their own occupancy.
 *  A body part is defined by two joints and its thickness.
 *  Body parts are used for the maximum velocity
 *  and acceleration based approaches
 *  and thus produce by two versions of occupancies:
 *  1. ArticulatedVel: Occupancy based on reachable space with constant velocity
 *  2. BodyPartAccel: Occuapancy based on live velocity measurements
 *     and constant maximum acceleration
 */
class BodyPart : public Occupancy {
 public:
  //! \brief Empty constructor
  BodyPart();

  //! \brief Instantiates a general body part
  //! \param[in] name Name of the body part
  //! \param[in] thickness Estimated thickness of the body part
  BodyPart(std::string name, double thickness);

  //! \brief Empty destructor
  ~BodyPart() {}

  //! \brief Returns the current occupancy
  inline Capsule get_occupancy() {
    return this->occupancy_;
  }

  //! \brief Returns the thickness of the body part
  double get_thicknes() {
      return this->thickness_;
  }

 protected:
  //! \brief Contains the current occupancy
  Capsule occupancy_ = Capsule();

  //! \brief Thickness of the body part
  double thickness_;
};
}  // namespace body_parts
}  //  namespace occupancies
#endif  //  REACH_LIB_INCLUDE_BODY_PART_HPP_

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

#include <map>
#include <string>
#include <utility>
#include <vector>

#include "articulated.hpp"
#include "body_part_accel.hpp"
#include "capsule.hpp"
#include "obstacle.hpp"
#include "system.hpp"

#ifndef REACH_LIB_INCLUDE_ARTICULATED_ACCEL_HPP_
#define REACH_LIB_INCLUDE_ARTICULATED_ACCEL_HPP_

namespace obstacles {
namespace articulated {
namespace accel {

//! \typedef A shortcut to the body part class for the ACCEL model
typedef occupancies::body_parts::accel::BodyPartAccel BodyPartAccel;

/*! This class defines our maximum acceleration based
 *  full body articulated reachable occupancy model.
 *  It recieves current Cartesian joint positions
 *  and velocity vectors and determines occupancy using
 *  static maximum acceleration parameters for all joints.
 */
class ArticulatedAccel : public Articulated {
 public:
  //! \brief Empty constructor
  ArticulatedAccel() : Articulated() {}

  //! \brief Instantiates the maximum acceleration based model from joint pairs.
  //! \param[in] system System parameters such as: delay and measurement errors
  //! \param[in] body_segment_map_ An association between joints and body segments
  //! \param[in] thickness Defines the thickness of each body part [body part name, thickness]
  //! \param[in] max_a Maximum acceleration of each joint
  ArticulatedAccel(System system, std::map<std::string, jointPair> body_segment_map,
                   const std::map<std::string, double>& thickness,
                   const std::vector<double>& max_a);

  //! \brief Empty destructor
  ~ArticulatedAccel() {}

  //! \brief Calcualtes the current occupancy using the Articuated 'ACCEL' model
  //! \param[in] p Current joint positions in Cartesian coordinartes (x, y ,z)
  //! \param[in] v Current joint velocities
  //! \param[in] t_a Start of the interval of analysis
  //! \param[in] t_b End of the interval of analysis
  std::vector<BodyPartAccel> update(double t_a, double t_b,
                                     std::vector<Point> p,
                                     std::vector<Point> v = {});

  //! \brief Returns true if the current occupancy intersects with
  //!        any given point in 'targets'
  //! \param[in] targets A list of points in global Cartesian coordinates (x, y, z)
  //!                    checked against the current occupancy
  bool intersection(std::vector<Point> targets);

  //! \brief Returns the mode of reachability analysis
  //!        of this class as 'ACCEL'
  std::string get_mode() {
      return "ARICULATED-ACCEL";
  }

  //! \brief Returns the current occupancy as a list of body parts
  std::vector<BodyPartAccel> get_occupancy() {
    return this->occupancy_;
  }

 private:
  //! \brief Contains the most recent state of all occupancy segments
  std::vector<BodyPartAccel> occupancy_ = {};
};
}  // namespace accel
}  // namespace articulated
}  // namespace obstacles
#endif  //  REACH_LIB_INCLUDE_ARTICULATED_ACCEL_HPP_

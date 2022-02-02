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
#include "extremity.hpp"
#include "capsule.hpp"
#include "obstacle.hpp"
#include "system.hpp"

#ifndef REACH_LIB_INCLUDE_ARTICULATED_POS_HPP_
#define REACH_LIB_INCLUDE_ARTICULATED_POS_HPP_

namespace obstacles {
namespace articulated {
namespace pos {

//! \typedef A shortcut to the extremity class for the POS model
typedef occupancies::extremities::Extremity Extremity;

/*! This class defines our purely maximum position based
 *  full body articulated reachable occupancy model.
 *  It recieves current Cartesian joint positions
 *  and determines occupancy using static maximum velocity parameters
 *  and the lengths of human limbs. It consist of four capsules (balls)
 *  centered at the shoulders and hips respectively.
 */
class ArticulatedPos : public Articulated {
 public:
  //! \brief Empty constructor
  ArticulatedPos() : Articulated() {}

  //! \brief Instantiates the position based model from joint pairs.
  //! \param[in] system System parameters such as: delay and measurement errors
  //! \param[in] body_segment_map_ An association between joints and body segments (extremities)
  //! \param[in] thickness Defines the thickness of each extreity [extremity name, thickness]
  //! \param[in] max_v Maximum velocity of each joint in order of body_segment_map
  //! \param[in] length Lengths of all extremities in order of body_segment_map
  ArticulatedPos(System system, std::map<std::string, jointPair> body_segment_map,
                 const std::vector<double>& thickness,
                 const std::vector<double>& max_v,
                 const std::vector<double>& length);

  //! \brief Empty destructor
  ~ArticulatedPos() {}

  //! \brief Calcualtes the current occupancy using the Articuated 'POS' model
  //! \param[in] p Current joint positions in Cartesian coordinartes (x, y ,z)
  //! \param[in] t_a Start of the interval of analysis
  //! \param[in] t_b End of the interval of analysis
  std::vector<Extremity> update(double t_a, double t_b,
                                std::vector<Point> p,
                                std::vector<Point> v = {});

  //! \brief Returns true if the current occupancy intersects with
  //!        any given point in 'targets'
  //! \param[in] targets A list of points in global Cartesian coordinates (x, y, z)
  //!                    checked against the current occupancy
  const bool intersection(std::vector<Point> targets);

  //! \brief Returns the mode of reachability analysis
  //!        of this class as 'ACCEL'
  std::string get_mode() {
      return "ARTICULATED-POS";
  }

  //! \brief Returns the current occupancy as a list of extremities
  std::vector<Extremity> get_occupancy() {
    return this->occupancy_;
  }

 private:
  //! \brief Contains the current occupancy
  std::vector<Extremity> occupancy_;
};
}  // namespace pos
}  // namespace articulated
}  // namespace obstacles
#endif  // REACH_LIB_INCLUDE_ARTICULATED_POS_HPP_

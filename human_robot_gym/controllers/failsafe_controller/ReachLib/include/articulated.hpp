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

#include "body_part.hpp"
#include "capsule.hpp"
#include "obstacle.hpp"
#include "occupancy.hpp"
#include "system.hpp"

#ifndef REACH_LIB_INCLUDE_ARTICULATED_HPP_
#define REACH_LIB_INCLUDE_ARTICULATED_HPP_

/*! This namespace contains the definition of our
 *  full body articulated reachable occupancy model.
*/
namespace obstacles {
namespace articulated {

//! \brief A shortcut to the BodyPart type
typedef occupancies::Occupancy Occupancy;

//! \brief A shortcut to the Capsule type
typedef occupancy_containers::capsule::Capsule Capsule;

/*! \typedef Defines a pair of two joints (may be the same) that each body segment
 *  (body part or extremity) is described by.
*/
typedef std::pair<int, int> jointPair;

/*! This class defines our full body articulated
 *  reachable occupancy model whereby three approaches
 *  to occupancy calculation are differentiated.
 *  They are based on:
 *  1. ArticulatedPos:   Length of extremities at maximum extension
 *  2. ArticulatedVel:   Occupancy based on reachable space with constant velocity
 *  3. ArticulatedAccel: Occuapancy based on live velocity measurements
 *     and constant maximum acceleration
*/
class Articulated : public Obstacle {
 public:
  //! \brief Empty constructor
  Articulated() : Obstacle() {}

  //! \brief Instantiates the articulated human model from joint pairs
  //! \param[in] system System parameters such as: delay and measurement errors
  //! \param[in] body_segment_map_ An association between joints and body segments
  Articulated(System system,
              std::map<std::string, jointPair> body_segment_map);

  //! \brief Empty destructor
  virtual ~Articulated() {}

  //! \brief Compute the current reachable occupancy of every body segment.
  //!        The function should be called within a verification loop.
  //! \param[in] p Position of all joints in Cartesian global coordinates (x, y, z)
  //! \param[in] v Velocity vector of all joints in Cartesian global coordinates (x, y, z)
  //! \param[in] t_a Start time of reachability analysis interval
  //! \param[in] t_b End time of reachability analysis interval
  std::vector<Occupancy> update(double t_a, double t_b,
                                     std::vector<Point> p,
                                     std::vector<Point> v = {}) {
    throw "Function safety_perimeters::articulated::Articulated::update is not defined!";
  }

  //! \brief Returns true if the current occupancy intersects with
  //!        any given point in 'targets'
  //! \param[in] targets A list of points in global Cartesian coordinates (x, y, z)
  //!                    checked against the current occupancy
  const bool intersection(std::vector<Point> targets) {
    throw "Function safety_perimeters::articulated::Articulated::intersection is not defined!";
  }

  //! \brief Returns the mode of the reachalility analysis:
  //!        Mode returned from: [ACCEL, VEL, POS]
  std::string get_mode() {
    std::cout << "\n" << "Got error!" << "\n";
    throw "Function safety_perimeters::articulated::Articulated::get_mode is not defined!";
  }

  //! \brief Returns the map of body segments and joint pairs
  std::map<std::string, articulated::jointPair> get_body_segment_map() {
    return this->body_segment_map_;
  }

 protected:
  //! \brief Associates body segments and their joint pairs
  std::map<std::string, articulated::jointPair> body_segment_map_;
};
}  //  namespace articulated
}  //  namespace obstacles
#endif  // REACH_LIB_INCLUDE_ARTICULATED_HPP_

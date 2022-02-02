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
#include "pedestrian.hpp"
#include "point.hpp"
#include "obstacle.hpp"
#include "system.hpp"

#ifndef REACH_LIB_INCLUDE_PEDESTRIAN_VEL_HPP_
#define REACH_LIB_INCLUDE_PEDESTRIAN_VEL_HPP_

namespace obstacles {
namespace pedestrian {
namespace vel {

//! \brief Defines a cylindrical safety perimeter based on
//! definiions in ISO10218: https://www.iso.org/standard/51330.html and
//! descriptions by http://mediatum.ub.tum.de/doc/1379662/905746.pdf.
//! Uses constant maximum velocity estimates for occupancy calculations.
class PedestrianVel : public Pedestrian {
 public:
  //! \brief Empty constructor
  PedestrianVel() : Pedestrian() {}

  //! \brief Instantiates a cylindrical safety perimeter
  //!        based on a constant maximum velocity
  //! \param[in] max_v Maximum velocity (constant)
  PedestrianVel(const System& sensor, double height,
                double arm_span, double max_v, double offset = 0.0,
                const Point& init_pos = Point());

  //! \brief Calcualtes the current occupancy using this model
  //! \param[in] p Current location of points of interest in Cartesian coordinartes (x, y ,z)
  //! \param[in] v Current velocity of points of interest [static velocity is used here]
  //! \param[in] t_a Start of the interval of analysis
  //! \param[in] t_b End of the interval of analysis
  std::vector<CylinderPerimeter> update(double t_a, double t_b,
                                     std::vector<Point> p,
                                     std::vector<Point> v = {});

  //! \brief Determines if any point from the list 'targets' is located inside
  //!        the current occupancy.
  //! \param[in] targets A list of points of interest to be checked against
  //!                    the current occupancy.
  const bool intersection(std::vector<Point> targets);

  //! \brief Returns the mode of reachability analysis
  //!        of this class as 'PEDESTRIAN-VEL'
  std::string get_mode() {
      return "PEDESTRIAN-VEL";
  }

  //! \brief Returns the maximum velocity parameter
  double get_max_v() {
    return this->max_v_;
  }

  //! \brief Emtpy destructor
  ~PedestrianVel() {}

 private:
  //! \brief Maximum velocity of the human (constant)
  double max_v_ = 0.0;
};
}  // namespace vel
}  // namespace pedestrian
}  // namespace obstacles
#endif  //  REACH_LIB_INCLUDE_PEDESTRIAN_VEL_HPP_

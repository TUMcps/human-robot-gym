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

#include "body_part.hpp"
#include "capsule.hpp"
#include "occupancy.hpp"
#include "point.hpp"

#ifndef REACH_LIB_INCLUDE_BODY_PART_VEL_HPP_
#define REACH_LIB_INCLUDE_BODY_PART_VEL_HPP_

namespace occupancies {
namespace body_parts {
namespace vel {

class BodyPartVel : public BodyPart {
 public:
  //! \brief Empty constructor
  BodyPartVel() : BodyPart() {}

  //! \brief Instatiates a BodyPart that updates based on
  //!        maximum estimated velocities.
  //! \param[in] name Name of the body part
  //! \param[in] thickness Estimated thickness of the body part
  //! \param[in] max_a Estimated maximum velocity of this body part
  BodyPartVel(std::string name, double thickness, double max_a1 = 0.0, double max_a2 = 0.0);

  //! \brief Empty destructor
  ~BodyPartVel() {}

  //! \brief Calcualtes the current occupancy using any model
  //! \param[in] p Current joint position in Cartesian coordinartes (x, y ,z)
  //! \param[in] v Unused overriden parameter
  //! \param[in] t_a Start of the interval of analysis
  //! \param[in] t_b End of the interval of analysis
  //! \param[in] measurement_error_pos Measurement errors in cartesian position [m]
  //! \param[in] measurement_error_vel Unused overriden parameter
  //! \param[in] delay Time delay within the executing system
  void update(const std::vector<Point>& p,
              const std::vector<Point>& v = {},
              double t_a = 0.0, double t_b = 0.0,
              double measurement_error_pos = 0.0,
              double measurement_error_vel = 0.0,
              double delay = 0.0);

  //! \brief Determines if any point from the list 'targets' is located inside
  //!        the current occupancy.
  const bool intersection(std::vector<Point> targets);

 private:
  //! \brief Maximum estimated velocity of the proximal joint (constant)
  double max_v1_ = 0.0;

  //! \brief Maximum estimated velocity of the distal joint (constant)
  double max_v2_ = 0.0;

  //! \brief Determines the ball occupancy within [0.0, t_end] for one joint
  //! This needs to be performed for both joints before enclosing the occupacnies
  //! in one capsule during the updated (carried out in update()).
  //! \param[in] p Origin of the ball
  //! \param[in] index Indicates the proximal (1) or distal (2) link
  //! \param[in] t_end End point of the time interval
  //! \param[in] delay Time delay within the executing system
  //! \param[in] measurement_error_pos Measurement errors in cartesian position [m]
  Capsule ry(const Point& p, int index, double t_end = 0.02,
             double delay = 0.0, double measurement_error_pos = 0.0);
};
}  // namespace vel
}  // namespace body_parts
}  // namespace occupancies
#endif  // REACH_LIB_INCLUDE_BODY_PART_VEL_HPP_

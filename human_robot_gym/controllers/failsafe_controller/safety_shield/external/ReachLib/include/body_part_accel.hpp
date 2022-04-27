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

#include "articulated.hpp"
#include "body_part.hpp"
#include "capsule.hpp"
#include "occupancy.hpp"
#include "point.hpp"

#ifndef REACH_LIB_INCLUDE_BODY_PART_ACCEL_HPP_
#define REACH_LIB_INCLUDE_BODY_PART_ACCEL_HPP_

namespace occupancies{
namespace body_parts {
namespace accel {

class BodyPartAccel : public BodyPart {
 public:
  //! \brief Empty constructor
  BodyPartAccel() : BodyPart() {}

  //! \brief Instatiates a BodyPart that updates based on live velocities
  //!        and maximum estimated acceleration.
  //! \param[in] name Name of the body part
  //! \param[in] thickness Estimated thickness of the body part
  //! \param[in] max_a12 Estimated maximum acceleration of this body part
  BodyPartAccel(std::string name, double thickness, double max_a1 = 0.0, double max_a2 = 0.0);

  //! \brief Empty destructor
  ~BodyPartAccel() {}

  //! \brief Calcualtes the current occupancy using the Articuated model.
  //! Overrides the identical blueprint method from Occupancy.
  //! \param[in] p Current joint position in Cartesian coordinartes (x, y ,z) size(p) = 2.
  //! \param[in] v Current joint velocity size(v) = 2.
  //! \param[in] t_a Start of the interval of analysis.
  //! \param[in] t_b End of the interval of analysis.
  //! \param[in] measurement_error_pos Measurement errors in cartesian position [m].
  //! \param[in] measurement_error_vel Measurement errors in cartesian velocity [m/s].
  //! \param[in] delay Time delay within the executing system.
  void update(const std::vector<Point>& p,
              const std::vector<Point>& v = {},
              double t_a = 0.0, double t_b = 0.0,
              double measurement_error_pos = 0.0,
              double measurement_error_vel = 0.0,
              double delay = 0.0);

  //! \brief Determines if any point from the list 'targets' is located inside
  //!        the current occupancy.
  bool intersection(std::vector<Point> targets);

 private:
  //! \brief Maximum estimated acceleration of the proximal joint (constant)
  double max_a1_ = 0.0;

  //! \brief Maximum estimated acceleration of the distal joint (constant)
  double max_a2_ = 0.0;

  //! \brief Determines the ball occupancy within [0.0, t_b] for one joint
  //! This needs to be performed for both joints before enclosing the occupacnies
  //! in one capsule during the updated (carried out in update()).
  //! \param[in] p Origin of the ball (Cartesian position of the joint)
  //! \param[in] v Velocity of the joint (Cartesian)
  //! \param[in] index Determines the joint of which ry is calculated; -> from [1,2].
  //! \param[in] t_b End point of the time interval
  //! \param[in] delay Time delay within the executing system
  //! \param[in] measurement_error_pos Measurement errors in cartesian position [m]
  //! \param[in] measurement_error_vel Measurement errors in cartesian velocity [m/s]
  Capsule ry(const Point& p, const Point& v,
             int index, double t_b = 0.02, double delay = 0.0,
             double measurement_error_pos = 0.0,
             double measurement_error_vel = 0.0);
};
}  // namespace accel
}  // namespace body_parts
}  // namespace occupancies
#endif  // REACH_LIB_INCLUDE_BODY_PART_ACCEL_HPP_

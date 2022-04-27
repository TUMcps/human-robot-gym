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

#ifndef REACH_LIB_INCLUDE_SYSTEM_HPP_
#define REACH_LIB_INCLUDE_SYSTEM_HPP_

namespace systems {

//! This class contains parameters that characterize properties
//! of the hardware and software system used for sensing and preparation of
//! position and velocity data.
class System {
 public:
  //! \brief Defines the positional measurement error of the underlying system
  double measurement_error_pos_ = 0.0;

  //! \brief Defines the measurement error of the underlying system for recorded velocities
  double measurement_error_vel_ = 0.0;

  //! \brief Defines the delay imposed by sensing and preparation of incoming signals
  double delay_ = 0.0;

  //! \brief Instantiates an object of type System
  //! \param[in] measurement_error_pos The positional measurement error
  //! of the underlying system
  //! \param[in] measurement_error_vel The measurement error of recorded velocities
  //! of the underlying system
  //! \param[in] delay The delay imposed by sensing and preparation of incoming signals
  System(double measurement_error_pos = 0.0,
         double measurement_error_vel = 0.0,
         double delay = 0.0);

  //! \brief Empty destructor
  ~System() {}
};
}  // namespace system
#endif  // REACH_LIB_INCLUDE_SYSTEM_HPP_

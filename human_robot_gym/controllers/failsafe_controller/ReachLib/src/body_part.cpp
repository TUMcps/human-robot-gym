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

namespace occupancies {
namespace body_parts {

BodyPart::BodyPart() : Occupancy() {
  this->occupancy_p = &this->occupancy_;
}

BodyPart::BodyPart(std::string name, double thickness) : thickness_(thickness), Occupancy(name) {
  //  NO TODO
}
}  // namespace body_parts
}  // namespace occupancies

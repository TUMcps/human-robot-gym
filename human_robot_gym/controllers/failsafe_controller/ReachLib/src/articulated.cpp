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
#include "capsule.hpp"
#include "obstacle.hpp"
#include "system.hpp"

namespace obstacles {
namespace articulated {
Articulated::Articulated(System system,
                         std::map<std::string, jointPair> body_segment_map) :
                         body_segment_map_(body_segment_map), Obstacle(system) {
  // NO TODO
}
}  // namespace articulated
}  // namespace obstacles

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
#include "pedestrian.hpp"
#include "point.hpp"
#include "obstacle.hpp"
#include "system.hpp"

namespace obstacles {
namespace pedestrian {

Pedestrian::Pedestrian(System sensor, double height, double arm_span, double offset,
                       const Point& init_pos) :
                       height_(height), arm_span_(arm_span), offset_(offset),
                       Obstacle(sensor) {
  this->occupancy_ = {CylinderPerimeter(init_pos, height, 0.0)};
}

}  // namespace pedestrian
}  // namespace obstacles

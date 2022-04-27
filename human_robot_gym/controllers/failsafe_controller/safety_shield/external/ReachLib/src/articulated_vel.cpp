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
#include "articulated_vel.hpp"
#include "body_part_vel.hpp"
#include "capsule.hpp"
#include "obstacle.hpp"
#include "system.hpp"


namespace obstacles {
namespace articulated {
namespace vel {

ArticulatedVel::ArticulatedVel(System system, std::map<std::string, jointPair> body_segment_map,
                                   const std::map<std::string, double>& thickness,
                                   const std::vector<double>& max_v) :
                                   Articulated(system, body_segment_map) {
  // Create a list of BodyPartsAccel that is later set as occpancy_
  std::vector<BodyPartVel> body = {};
  for (const auto& it : body_segment_map) {
    body.push_back(BodyPartVel(it.first, thickness.at(it.first), max_v.at(it.second.first),
                                 max_v.at(it.second.second)));
  }
  this->occupancy_ = body;
  // Initialize pointers
  for (int i = 0; i < this->occupancy_.size(); i++) {
    this->occupancy_p.push_back(&(this->occupancy_[i]));
  }
}

std::vector<BodyPartVel> ArticulatedVel::update(double t_a, double t_b,
                                                std::vector<Point> p,
                                                std::vector<Point> v) {
  int count = 0;
  for (auto& it : this->occupancy_) {
    int p1_id = this->body_segment_map_.at(it.get_name()).first;
    int p2_id = this->body_segment_map_.at(it.get_name()).second;
    it.update({p[p1_id], p[p2_id]}, {}, t_a, t_b,
              this->system.measurement_error_pos_, 0.0,
              this->system.delay_);
    this->occupancy_[count] = it;
    count++;
  }
  return this->occupancy_;
}

const bool ArticulatedVel::intersection(std::vector<Point> targets) {
  for (auto& it : this->occupancy_) {
    it.intersection(targets);
  }
}
}  // namespace vel
}  // namespace articulated
}  // namespace obstacles

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
#include "pedestrian_accel.hpp"
#include "point.hpp"
#include "obstacle.hpp"
#include "system.hpp"


namespace obstacles {
namespace pedestrian {
namespace accel {

PedestrianAccel::PedestrianAccel(const System& system, double height,
                                 double arm_span, double max_a, double offset,
                                 const Point& init_pos, int steps) :
                                 Pedestrian(system, height, arm_span, offset, init_pos),
                                 max_a_(max_a), steps_(steps) {
  this->occupancy_p.push_back(&(this->occupancy_[0]));
}

std::vector<CylinderPerimeter> PedestrianAccel::update(double t_a, double t_b,
                                                       std::vector<Point> p,
                                                       std::vector<Point> v) {
  // Reset cylinder list
  this->cylinder_list_ = {};
  double scale = 0.0;

  // Creating a container for the solution of type Cylinder
  Cylinder solution = Cylinder();

  // Setting current position
  Point pos = p[0];
  Point vel = v[0];

  // Calculating solution over desired interval
  // Calculating the cylinder of [0.0, t_a]
  // Get x and y position from equations of motion x = x0 + xd*t + 0.5*xdd*t^2
  double x_s = pos.x + this->system.measurement_error_pos_ +
               (vel.x + this->system.measurement_error_vel_) * (t_a + this->system.delay_) +
               this->max_a_ * pow((t_a + this->system. delay_), 2);
  double y_s = pos.y + this->system.measurement_error_pos_ +
               (vel.y + this->system.measurement_error_vel_) * (t_a + this->system.delay_) +
               this->max_a_ * pow((t_a + this->system.delay_), 2);

  double accel_rad_s = 0.5 * this->arm_span_ + this->system.measurement_error_pos_ +
                       (Point::norm(Point(vel.x, vel.y,0.0)) + this->system.measurement_error_vel_) * (t_a + this->system.delay_) +
                       0.5 * this->max_a_ * pow((t_a + this->system.delay_), 2);

  CylinderPerimeter accel_cyl_s = CylinderPerimeter(Point(x_s, y_s, 0.0), this->height_, accel_rad_s);

  // Calculating the cylinder of [0.0, t_b]
  double x_e = pos.x + this->system.measurement_error_pos_ +
               (vel.x + this->system.measurement_error_vel_) * (t_b + this->system.delay_) +
               this->max_a_*pow((t_b + this->system.delay_), 2);

  double y_e = pos.y + this->system.measurement_error_pos_ +
               (vel.y + this->system.measurement_error_vel_) * (t_b + this->system.delay_) +
               this->max_a_ * pow((t_b + this->system.delay_), 2);

  double accel_rad_e = 0.5 * this->arm_span_ + this->system.measurement_error_pos_ +
                       (Point::norm(Point(vel.x , vel.y, 0.0)) + this->system.measurement_error_vel_) * (t_b + this->system.delay_) +
                       0.5 * this->max_a_ * pow((t_b + this->system.delay_), 2);

  Cylinder accel_cyl_e = Cylinder(Point(x_e, y_e, 0.0), Point(x_e, y_e, this->height_), accel_rad_e);

  // Calculating the ball/circle enclosure of two cylinders at t_a and t_b
  Point p_diff = Point(x_e, y_e, 0.0) - Point(x_s, y_s, 0.0);
  double norm_diff = Point::norm(p_diff);
  double beta = std::min(accel_rad_e - accel_rad_s, norm_diff);
  double alpha = std::max(accel_rad_e - accel_rad_s, norm_diff);
  Point pk;
  if (norm_diff == 0.0) {
    pk = Point(x_s, y_s, 0.0);
  } else {
    pk = Point(x_s + (p_diff.x/norm_diff) * beta, y_s + (p_diff.y/norm_diff) * beta, 0.0);
  }
  Point origin = Point((x_e + pk.x)/2.0, (y_e + pk.y)/2.0, this->offset_);
  double radius = 0.5*(accel_rad_s + accel_rad_e + alpha);
  this->occupancy_ = {CylinderPerimeter(origin, this->height_, radius)};

  this->cylinder_list_.push_back(Cylinder(Point(origin.x, origin.y, origin.z),
                                 Point(origin.x, origin.y, origin.z), radius));

  // Calculating fixed step iterations for the sub-occupancies
  for (int i = 0; i < this->steps_; i++) {
      scale = static_cast<double>(i) / static_cast<double>(this->steps_);
      double x = pos.x + this->system.measurement_error_pos_ +
                 (vel.x + this->system.measurement_error_vel_) * scale * (t_b + this->system.delay_) +
                 this->max_a_ * pow(scale*(t_b+this->system.delay_), 2);

      double y = pos.y + this->system.measurement_error_pos_ +
                 (vel.y + this->system.measurement_error_vel_) * scale * (t_b+this->system.delay_) +
                 this->max_a_ * pow(scale*(t_b+this->system.delay_), 2);

      double accel_rad = 0.5 * this->arm_span_ + this->system.measurement_error_pos_ + (Point::norm(vel) +
                         this->system.measurement_error_vel_) * scale * (t_b+this->system.delay_) +
                         0.5*this->max_a_*pow(scale*(t_b+this->system.delay_), 2);

      Cylinder accel_cyl = Cylinder(Point(x, y, this->offset_), Point(x, y, this->height_), accel_rad);
      this->cylinder_list_.push_back(accel_cyl);
  }
  return this->occupancy_;
}

const bool PedestrianAccel::intersection(std::vector<Point> targets) {
  bool intersection = false;
  for (const auto& it : targets) {
    intersection = this->occupancy_[0].intersection(targets);
  }
  return intersection;
}


}  // namespace accel
}  // namespace pedestrian
}  // namespace obstacles

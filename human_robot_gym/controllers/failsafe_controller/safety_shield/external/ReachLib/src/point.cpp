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

#include <cmath>
#include <tuple>

#include "point.hpp"

namespace point {

Point::Point(double x, double y, double z) : x(x), y(y), z(z) {
  //  NO TODO
}

Point Point::origin(const Point& p1, const Point& p2) {
  Point p = p1 - p2;
  p.x = p2.x + p.x/2.0;
  p.y = p2.y + p.y/2.0;
  p.z = p2.z + p.z/2.0;
  return p;
}

double Point::norm(const Point& p) {
  return sqrt(pow(p.x, 2) + pow(p.y, 2) + pow(p.z, 2));
}

double Point::norm(const Point& p1, const Point& p2) {
  return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2));
}

double Point::inner_dot(const Point& p1, const Point& p2) {
  return p1.x * p2.x + p1.y * p2.y + p1.z * p2.z;
}

Point Point::cross(const Point& p1, const Point& p2) {
  Point c = Point();
  c.x = p1.y*p2.z - p1.z*p2.y;
  c.y = p1.z*p2.x - p1.x*p2.z;
  c.z = p1.x*p2.y - p1.y*p2.x;
  return c;
}

double Point::determinant3x3(Point X, Point Y, Point Z) {
  // A = [X, Y, Z] = [[a b c], [d e f], [g h i]]
  // det(A) = a(ei - fh) - b(di - fg) + c(dh - eg)
  return X.x * (Y.y * Z.z - Y.z * Z.y) -
         X.y * (Y.x * Z.z - Y.z * Z.x) +
         X.z * (Y.x * Z.y - Y.y * Z.x);
}

// legacy (not required)
quaternion Point::orientation(const Point& p1, const Point& p2) {
  Point p1_loc = p1;
  Point p2_loc = p2;
  if (p2.z < p1.z) {
      Point temp = p1;
      p1_loc = p2_loc;
      p2_loc = temp;
  }

  double dis = norm(p1_loc, p2_loc);
  double f = 0.0174533;
  double r = 0.0;
  double p = 0.0;
  double y = 0.0;

  if (dis == 0) {
      p = 0.0;
  } else {
      p = -asin((p2_loc.y - p1_loc.y) / dis);
  }

  if (cos(p) == 0.0 ||dis == 0.0) {
      y = 0.0;
  } else {
      y = asin((p2_loc.x - p1_loc.x)/(cos(p) * dis));
  }

  // capsules are roll invariant

  double cy = cos(y * 0.5);
  double sy = sin(y * 0.5);
  double cp = cos(p * 0.5);
  double sp = sin(p * 0.5);
  double cr = cos(r * 0.5);
  double sr = sin(r * 0.5);

  //  q := (x,y,z,w)
  //  q.w = cr * cp * cy + sr * sp * sy;
  //  q.x = sr * cp * cy - cr * sp * sy;
  //  q.y = cr * sp * cy + sr * cp * sy;
  //  q.z = cr * cp * sy - sr * sp * cy;

  return std::make_tuple(sr * cp * cy - cr * sp * sy,
                          cr * sp * cy + sr * cp * sy,
                          cr * cp * sy - sr * sp * cy,
                          cr * cp * cy + sr * sp * sy);
}
}  //  namespace point

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

#ifndef REACH_LIB_INCLUDE_POINT_HPP_
#define REACH_LIB_INCLUDE_POINT_HPP_

namespace point {

//! \typedef A local definition for quaternions (x, y, z, w).
typedef std::tuple<double, double, double, double> quaternion;

//! This class describes points in
//! three dimensional Cartesian coordinates (x, y, z).
//! It further implements typical Cartesian vector functions.
class Point {
 public:
  //! \brief X coordinate
  double x = 0.0;

  //! \brief X coordinate
  double y = 0.0;

  //! \brief X coordinate
  double z = 0.0;

  //! \brief Empty constructor
  Point() {}

  //! \brief Instanciates a point from three coordinate inputs
  //! \param[in] x X coordinate
  //! \param[in] y Y coordinate
  //! \param[in] z Z coordinate
  Point(double x, double y, double z);

  //! \brief Empty destructor
  ~Point() {}

  //! \brief Returns the midpoint on the line defined by p1 and p2.
  //! \param[in] p1 Starting point of the line
  //! \param[in] p2 End point of the line
  static Point origin(const Point& p1, const Point& p2);

  //! \brief Returns the L2 norm of the vector from origin to p1.
  //! \param[in] p1 Defins the endpoint of the vector OP1.
  static double norm(const Point& p1);

  //! \brief Returns the L2 norm of the vector from p1 to p2.
  //! \param[in] p1 Starting point of the vector
  //! \param[in] p2 End point of the vector
  static double norm(const Point& p1, const Point& p2);

  //! \brief Defines the inner dot product between the
  //!        points p1 and p2 as given: https://en.wikipedia.org/wiki/Dot_product
  //! \param p1 First origin vector
  //! \param p2 Second origin vector
  static double inner_dot(const Point& p1, const Point& p2);

  //! \brief Defines the cross product between the
  //!        points p1 and p2 as given: https://en.wikipedia.org/wiki/Cross_product
  //! \param p1 First origin vector
  //! \param p2 Second origin vector
  static Point cross(const Point& p1, const Point& p2);

  //! \brief Calculates the determinante of a 3x3 matrix given by three points a row vectors
  //! \param[in] X First row vector
  //! \param[in] Y Second row vector
  //! \param[in] Z Third row vector
  static double determinant3x3(Point X, Point Y, Point Z);

  //! \brief Defines the orientation of the line starting at
  //!        p1 and ending in p2 as given by: https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation
  //! \param p1 Start point
  //! \param p2 End point
  static quaternion orientation(const Point& p1, const Point& p2);

  //! \brief Defines the '==' operator for two points
  //!        where all coordinates of both points must be equal
  //!        to produce 'true'.
  friend bool operator== (const Point& p1, const Point& p2) {
    if (p1.x == p2.x && p1.y == p2.y && p1.z == p2.z) {
      return true;
    } else {
      return false;
    }
  }

  //! \brief Defines the '-' operator for two points
  //!        where all coordinates of point 2 are subtracted from point 1.
  //! \param p1 Point that is subtracted from
  //! \param p2 Point whose coordinates are subtracted from p1
  friend Point operator- (const Point& p1, const Point& p2) {
    return Point(p1.x - p2.x, p1.y - p2.y, p1.z - p2.z);
  }

  //! \brief Defines the '+' operator for two points
  //!        where all coordinates of point 2 are added to point 1.
  //! \param p1 Point that is added to
  //! \param p2 Point whose coordinates are added to p1
  friend Point operator+ (const Point& p1, const Point& p2) {
    return Point(p1.x + p2.x, p1.y + p2.y, p1.z + p2.z);
  }
};
}  //  namespace point
#endif  //  REACH_LIB_INCLUDE_POINT_HPP_

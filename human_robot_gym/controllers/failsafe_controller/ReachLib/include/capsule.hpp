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

#include <tuple>
#include <vector>

#include "cylinder.hpp"
#include "occupancy_container.hpp"
#include "point.hpp"
#include "sphere.hpp"

#ifndef REACH_LIB_INCLUDE_CAPSULE_HPP_
#define REACH_LIB_INCLUDE_CAPSULE_HPP_

namespace occupancy_containers {
namespace capsule {

//! This class defines the capsule occupancy medium
//! as defined by Pereira et al.
//! [https://doi-org.eaccess.ub.tum.de/10.1109/IROS.2017.8206314].
//! It is given as two half spheres added to the top and bottom
//! of a cylinder defined by two points and a radius.
//! The capsule is classifies as a ball if the points coincide.
class Capsule : public OccupancyContainer {
 public:
  //! \brief Radius of the capsule
  double r_ = 0.0;

  //! \brief Center of the cylinders initial top circle
  Point p1_ = Point();

  //! \brief Center of the cylinders initial bottom circle
  Point p2_ = Point();

  //! \brief Empty constructor
  Capsule() : OccupancyContainer() {}

  //! \brief Constructs a Capsule from two points
  //!        and a radius.
  //! \param[in] p1 Center of the bottom cylinder circle
  //! \param[in] p2 center of the top cylindercircle
  //! \param[in] radius The radius of the capsule
  Capsule(const Point& p1, const Point& p2, double r);

  //! \brief Constructs a Capsule from a Sphere.
  //! Used for static_cast purposes only!
  //! \param[in] s An object of type Sphere
  explicit Capsule(const sphere::Sphere& s);

  //! \brief Constructs a Capsule from a Cylinder.
  //! Used for static_cast purposes only!
  //! \param[in] s An object of type Cylinder
  explicit Capsule(const cylinder::Cylinder& c);

  //! \brief Static cast constructor from Capsule to OccupancyModels
  explicit Capsule(const OccupancyContainer& o);

  //! \brief Empty destructor
  ~Capsule() {}

  //! \brief Determines whether the capsule intersects with
  //!        any point in points.
  //! \param[in] targets List of points of interest in Cartesian (x, y, z).
  bool intersection(const std::vector<Point>& targets);

  //! \brief Calculates the minkowski sum of two capsules.
  //!        It adds both points and radii of the two capsules.
  //! \param[in] c1 First capsule
  //! \param[in] c2 Second capsule
  static Capsule minkowski(const Capsule& c1, const Capsule& c2);

  //! \brief Calculates the capsule enclosing both input capsules
  //! \param[in] c1 First capsule
  //! \param[in] c2 Second capsule
  static Capsule capsuleEnclosure(const Capsule& c1, const Capsule& c2);

  //! \brief Calculates the ball enclosing both input capsules.
  //! \param[in] c1 First capsule; must be a ball.
  //! \param[in] c2 Second capsule; must be a ball.
  static Capsule ballEnclosure(const Capsule& c1, const Capsule& c2);

 private:
  //! \brief Returns a pair of the alpha/beta values defiend in:
  //! https://doi-org.eaccess.ub.tum.de/10.1109/IROS.2017.8206314
  //! \param[in] ri Radius of the ball with the greater radius
  //! \param[in] rj Radius of the ball with the lesser radius
  static std::tuple<double, double> alphaBeta(double ri, double rj, Point x);

  //! \brief Determines whether any 3D point in a list lies within a capsule.
  //! \param[in] c Capsule object checked for intersection
  //! \param[in] points A list of points in 3D Cartesian coordinates (x, y, z)
  //! \param[in] r Minmum safety distance between the point and the capsule.
  //!            Used to calculate ball capsule intersection.
  static bool point_capsule_intersection(const Capsule& c, const std::vector<Point>& points, double r = 0.0);

  //! \brief Calculates pk as defined in:
  //! https://doi-org.eaccess.ub.tum.de/10.1109/IROS.2017.8206314
  //! \param[in] pj Origin of the ball with the greater radius;
  //!               The origin of the ball with the lesser radius
  //!               is named pi
  //! \param[in] x  Difference vector of the ball origins
  //! \param[in] b  The beta value from alpha_beta
  static Point pk(Point pj, Point x, double b);
};
}  //  namespace capsule
}  //  namespace occupancy_containers
#endif  //  REACH_LIB_INCLUDE_CAPSULE_HPP_

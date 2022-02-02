#include "safety_shield/verify.h"

namespace safety_shield {

double Verify::segmentsDistance(const custom_robot_msgs::Segment &seg1, const custom_robot_msgs::Segment &seg2) {
  geometry_msgs::Point d1 = geometry_helpers::fromSegmentToVector(seg1.p,seg1.q);
  geometry_msgs::Point d2 = geometry_helpers::fromSegmentToVector(seg2.p,seg2.q);
  geometry_msgs::Point r = geometry_helpers::fromSegmentToVector(seg2.p,seg1.p);
  double a = geometry_helpers::scalarProduct(d1,d1);
  double e = geometry_helpers::scalarProduct(d2,d2);
  double f = geometry_helpers::scalarProduct(d2,r);
  double epsilon = 1e-6;

  double t,s;
  if(a <= epsilon && e <= epsilon) { //[p1,q1] is actually a point && [p2,q2] is actually a point
    return geometry_helpers::scalarProduct(r,r); // distance = norm(p2-p1)
  }
  else{
    double c = geometry_helpers::scalarProduct(d1,r);
    if (a <= epsilon) {
      s = 0;
      t = std::clamp(f/e, 0.0, 1.0);
    } else if(e < epsilon) {
      t = 0;
      s = std::clamp(-c/a, 0.0, 1.0);
    } else{
      double b = geometry_helpers::scalarProduct(d1,d2);
      double denom = a*e - b*b;
      if(denom != 0) {
        s = std::clamp((b*f - c*e)/denom, 0.0, 1.0);
      }
      else{
        s = 0;
      }
      t = (b*s + f)/e;
      if(t < 0) {
        t = 0;
        s = std::clamp(-c/a, 0.0, 1.0);
      }
      else if(t > 1) {
        t = 1;
        s = std::clamp((b - c)/a, 0.0, 1.0);
      }
    }
  
    geometry_msgs::Point closest_point_1 = geometry_helpers::getPointFromSegment(seg1,s);
    geometry_msgs::Point closest_point_2 = geometry_helpers::getPointFromSegment(seg2,t);
    return pointPointDistance(closest_point_1, closest_point_2);
  }
}
} // namespace safety_shield
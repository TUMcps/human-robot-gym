#include "global_library/global_library.h"

namespace geometry_helpers {

double distance2(const geometry_msgs::Point &point, const geometry_msgs::Point &p, const geometry_msgs::Point &q) {
  geometry_msgs::Point v = fromSegmentToVector(p,q);
  geometry_msgs::Point w = fromSegmentToVector(p,point);
  double e = scalarProduct(v,v);
  double f = scalarProduct(v,w);

  if(e < 1e-6) { // the segment is actually a point
    return scalarProduct(w,w); // dist = norm(p2-p1)
  }
  double t = std::clamp(f/e, 0.0, 1.0);
  custom_robot_msgs::Segment seg;
  seg.p = p;
  seg.q = q;
  geometry_msgs::Point closest_point_2 = getPointFromSegment(seg, t);
  geometry_msgs::Point closest_vector = fromSegmentToVector(point, closest_point_2);
  return scalarProduct(closest_vector, closest_vector);
}
} // namespace geometry_helpers



namespace polygon_helpers {

Vertices::Vertices(const std::vector<geometry_msgs::Point> &points) {
  double delta_x1 = (points.end() - 1)->x - points.begin()->x,
         delta_y1 = (points.end() - 1)->y - points.begin()->y,
         delta_x2,delta_y2;
  for(auto point = points.begin(); point != points.end() -1; point++) {
    delta_x2 = (point + 1)->x - point->x;
    delta_y2 = (point + 1)->y - point->y;
    // point can be an ear iif the angle between point-1, point and point+1 is acute
    this->vertices_.push_back(Vertice(*point, delta_x1 * delta_y2 - delta_x2 * delta_y1 > 0));
  }
  delta_x2 = points.begin()->x - (points.end() -1)->x;
  delta_y2 = points.begin()->y - (points.end() -1)->y;
  this->vertices_.push_back(Vertice(*(points.end() -1), delta_x1 * delta_y2 - delta_x2 * delta_y1 > 0));
}


bool Vertices::inTriangle(const std::vector<Vertice>::iterator &center, const  std::vector<Vertice>::iterator &point) {
  auto p1 = previous(center);
  auto p3 = next(center);
  if(p1 == point || p3 == point) return false;

  // 1) computes the barycentric coordinates of point
  geometry_msgs::Point u21 = geometry_helpers::fromSegmentToVector(center->p_, p1->p_),
                       u23 = geometry_helpers::fromSegmentToVector(center->p_, p3->p_),
                       v   = geometry_helpers::fromSegmentToVector(center->p_, point->p_);
  double norm21      = geometry_helpers::scalarProduct(u21, u21),
         norm23      = geometry_helpers::scalarProduct(u23, u23),
         u21_dot_u23 = geometry_helpers::scalarProduct(u21, u23),
         u21_dot_v   = geometry_helpers::scalarProduct(u21, v),
         u23_dot_v   = geometry_helpers::scalarProduct(u23, v);

  double denom = norm21*norm23 - pow(u21_dot_u23,2);
  double x = (norm21*u23_dot_v - u21_dot_u23*u21_dot_v)/denom;
  double y = (norm23*u21_dot_v - u21_dot_u23*u23_dot_v)/denom;

  // 2) check if the barycentric coordinates show that the point is in the triangle
  return (x >= 0 && y >= 0 && x + y < 1);
}


bool Vertices::isEar(const std::vector<Vertice>::iterator &vertice) {
  if(!vertice->is_candidate_) return false;

  for(auto point = vertices_.begin(); point != vertices_.end(); point++) {
    // a vertice is an ear if all the points are outside of the triangle (only the "obtuse vertices" can be inside)
    if(!point->is_candidate_ && inTriangle(vertice, point)) {
      return false;
    }
  }
  return true;
}


void Vertices::triangles(std::vector<geometry_msgs::Point> &triangles) {
  auto next_ear = vertices_.begin();
  while(vertices_.size() > 3) { // there are still more than one triangle
    // searches for the next ear
    while(!isEar(next_ear)) {
      next_ear++;
    }
    // add the new ear in the triangle list
    auto p1 = previous(next_ear);
    auto p3 = next(next_ear);
    triangles.push_back(p1->p_);
    triangles.push_back(next_ear->p_);
    triangles.push_back(p3->p_);

    // only p1 and p3 can have a state changment
    if(!p1->is_candidate_) {
      auto p0 = previous(p1);
      double delta_x1 = p0->p_.x - p1->p_.x;
      double delta_y1 = p0->p_.y - p1->p_.y;
      double delta_x3 = p3->p_.x - p1->p_.x;
      double delta_y3 = p3->p_.y - p1->p_.y;
      p1->is_candidate_ = delta_x1*delta_y3 - delta_x3*delta_y1 > 0;
    }
    if(!p3->is_candidate_) {
      auto p4 = next(p3);
      double delta_x2 = p1->p_.x - p3->p_.x;
      double delta_y2 = p1->p_.y - p3->p_.y;
      double delta_x4 = p4->p_.x - p3->p_.x;
      double delta_y4 = p4->p_.y - p3->p_.y;
      p1->is_candidate_ = delta_x2*delta_y4 - delta_x4*delta_y2 > 0;
    }

    // the next ear can be p1 or one of the next vertices of the list
    if(next_ear == vertices_.begin()) {
      vertices_.erase(next_ear);
      next_ear = vertices_.begin();
    }
    else{
      vertices_.erase(next_ear);
      next_ear = p1;
    }
  }
  triangles.push_back(previous(next_ear)->p_);
  triangles.push_back(next_ear->p_);
  triangles.push_back(next(next_ear)->p_);
}

std::vector<geometry_msgs::Point> getClockwisePolygon(const std::vector<custom_robot_msgs::Segment> &polygon)
{
  std::vector<geometry_msgs::Point> points;
  std::vector<custom_robot_msgs::Segment> poly;
  for(auto segment = polygon.begin(); segment != polygon.end(); segment++) {
    poly.push_back(*segment);
  }
  if(poly.size() == 1) {
    points.push_back(poly.begin()->p);
    points.push_back(poly.begin()->q);
    return points;
  }
  if(isClockwise(poly)) {
    for(auto segment = poly.begin(); segment != poly.end(); segment++) {
      points.push_back(segment->p);
    }
  }
  else{
    for(auto segment = poly.end()-1; segment != poly.begin(); segment++) {
      points.push_back(segment->p);
    }
    points.push_back(poly.begin()->p);
  }
  return points;
}
} // namespace polygon_helpers
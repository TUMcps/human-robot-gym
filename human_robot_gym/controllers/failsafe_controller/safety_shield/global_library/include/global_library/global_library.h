// -*- lsst-c++ -*/
/**
 * @file global_library.h
 * @brief Defines the calculation helper functions for reachability analysis
 * @version 0.1
 * @copyright MIT License. Please see package.xml for further detail.
 */

#include <string>
#include <cmath>
#include <algorithm>
#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <custom_robot_msgs/Segment.h>

#ifndef GLOBAL_LIBRARY_H
#define GLOBAL_LIBRARY_H

namespace geometry_helpers {
/**
 * @brief Prints the point p: (p.x,p.y)
 */
inline void print(const geometry_msgs::Point &p) {
  std::cout << "(" << p.x << "," << p.y << ")";
}

/**
 * @brief Constructs and returns a geometry_msgs::Point with the given coordinates (and z = 0)
 *
 * @param x the abscissa of the point
 * @param y the ordinate of the point
 * @return p(x,y,0)
 */
inline geometry_msgs::Point geometry_msgs(double x, double y) {
  geometry_msgs::Point p;
  p.x = x;
  p.y = y;
  p.z = 0;
  return p;
}

/**
 * @brief Constructs and returns a geometry_msgs::Point with the given coordinates 
 *
 * @param x the abscissa of the point
 * @param y the ordinate of the point
 * @param z the ordinate of the point
 * @return p(x,y,z)
 */
inline geometry_msgs::Point geometry_msgs(double x, double y, double z) {
  geometry_msgs::Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

/**
 * @brief Constructs and returns a custom_robot_msgs::Segment with the given coordinates 
 *
 * @param p1 the first point of the segment
 * @param p2 the second point of the segment
 * @return seg(p1,p2)
 */
inline custom_robot_msgs::Segment Segment(geometry_msgs::Point p1, geometry_msgs::Point p2) {
  custom_robot_msgs::Segment seg;
  seg.p = p1;
  seg.q = p2;
  return seg;
} 

/**
 * @brief Computes and returns the directory vector of a given segment (represented by two points)
 * 
 * @param p the first point of the segment
 * @param q the second point of the segment
 * @return the vector [p,q]
 */
inline geometry_msgs::Point fromSegmentToVector(const geometry_msgs::Point &p, const geometry_msgs::Point &q) {
  return geometry_msgs(q.x - p.x, q.y - p.y, q.z - p.z);
}
  
/**
 * @brief Computes and returns the point of a given segment with a given relative coordinate
 * 
 * @param seg the segment where the point should be
 * @param t the relative coordinate of the point on the segment seg
 * @return seg.p + (seg.q-seg.p)*t
 */
inline geometry_msgs::Point getPointFromSegment(const custom_robot_msgs::Segment &seg, double t) {
  return geometry_msgs(seg.p.x + (seg.q.x - seg.p.x)*t, 
      seg.p.y + (seg.q.y - seg.p.y)*t, 
      seg.p.z + (seg.q.z - seg.p.z)*t);
}
  
/**
 * @brief Computes and returns the sum of two given points
 * 
 * @param p1 the first point
 * @param p2 the second point
 * @return p1 + p2
 */
inline geometry_msgs::Point pointAdd(const geometry_msgs::Point &p1, const geometry_msgs::Point &p2) {
  return geometry_msgs(p1.x + p2.x, p1.y + p2.y, p1.z + p2.z); 
}

/**
 * @brief Computes and returns the sum of two given points and norms it
 * 
 * @param p1 the first point
 * @param p2 the second point
 * @param n  the value of the norm
 * @return (p1 + p2)/n
 */
inline geometry_msgs::Point pointAdd(const geometry_msgs::Point &p1, const geometry_msgs::Point &p2, double n) {
  geometry_msgs::Point p = pointAdd(p1,p2);
  p.x /= n;
  p.y /= n;
  p.z /= n;
  return p; 
}
  
/**
 * @brief Checks if two points are equals
 * 
 * @param p1 the first point
 * @param p2 the second point
 * @return p1 == p2
 */
inline double equal(const geometry_msgs::Point &p1, const geometry_msgs::Point &p2) {
  return (p1.x == p2.x && p1.y == p2.y && p1.z == p2.z);
}

/**
 * @brief Computes and returns the scalar product of two vectors
 *
 * @param u the first vector
 * @param v the second vector
 * @return u.v
 */
inline double scalarProduct(const geometry_msgs::Point &u, const geometry_msgs::Point &v) {
  return u.x*v.x + u.y*v.y + u.z*v.z;
}
  
/**
 * @brief Computes and returns the norm of a given vector
 * 
 * @param u the vector to norm
 * @return norm(u)
 */
inline double norm(const geometry_msgs::Point &u) {
  return sqrt(scalarProduct(u, u));
}

/**
 * @brief Computes and returns the cross product of two vectors
 * 
 * @param v1 first vector
 * @param v2 second vector
 * @return cross product vector
 */
inline geometry_msgs::Point cross(const geometry_msgs::Point& v1, const geometry_msgs::Point& v2) {
  geometry_msgs::Point p1;
  p1.x = v1.y*v2.z-v1.z*v2.y;
  p1.y = v1.z*v2.x-v1.x*v2.z;
  p1.z = v1.x*v2.y-v1.y*v2.x;
  return p1;
}

/**
 * @brief Computes and returns the square of the distance between two given points
 * 
 * @param p1 the first point
 * @param p2 the second point
 * @return dist(p1,p2)²
 */
inline double distance2(const geometry_msgs::Point &p1, const geometry_msgs::Point &p2) {
  return pow((p1.x - p2.x),2) + pow((p1.y - p2.y),2) + pow((p1.z - p2.z),2);
}

/**
 * @brief Computes and returns the the distance between two given points
 * 
 * @param p1 the first point
 * @param p2 the second point
 * @return dist(p1,p2)
 */
inline double distance(const geometry_msgs::Point &p1, const geometry_msgs::Point &p2) {
  return sqrt(distance2(p1, p2));
}

/**
 * @brief Computes and returns the square of the distance between a point and a segment (represented by two points)
 * 
 * @param point the point
 * @param p     the first point of the segment
 * @param q     the second point of the segment
 * @return dist(point,[p,q])²
 */
double distance2(const geometry_msgs::Point &point, const geometry_msgs::Point &p, const geometry_msgs::Point &q);
} // namespace geometry_helpers

namespace polygon_helpers {

/**
 * @brief Class that represents a polygon's vertice, used for the triangles extraction computation
 */
class Vertice {
 public:
  /**
   * @brief the coordinates of the vertice
   */
  geometry_msgs::Point p_;

  /**
   * @brief true iff the vertice can be the center of one ear
   */
  bool is_candidate_;
  
  /**
   * @brief A basic vertice constructor
   */
  Vertice(const geometry_msgs::Point &p, bool is_candidate):
   p_(p),
   is_candidate_(is_candidate) {}
};

/**
 * @brief Class that represents a polygon by its vertices
 */
class Vertices {
 private:
  std::vector<Vertice> vertices_;
  
  /**
   * @brief Returns the previous vertice of a given vertice of the polygon
   * @param vertice the vertice 
   * @return prev(vertice)
   */
  inline std::vector<Vertice>::iterator previous(std::vector<Vertice>::iterator vertice) {
    if(vertice == vertices_.begin()) {
      return vertices_.end() - 1;
    }
    return (vertice - 1);
  }
  
  /**
   * @brief Returns the next vertice of a given vertice of the polygon
   * @param vertice the vertice 
   * @return next(vertice)
   */
  std::vector<Vertice>::iterator next(std::vector<Vertice>::iterator vertice) {
    if(vertice == vertices_.end() - 1) {
      return vertices_.begin();
    }
    return (vertice + 1);
  }
  
  /**
   * @brief Check if one given point of the polygon is inside the triangle formed by three given consecutive points of the polygon
   * 
   * @param center the center of the triangle (the triangle is center-1, center, center+1)
   * @param point  the point to check 
   * @return true iff in(point, [center-1,center,center+1])
   */
  bool inTriangle(const std::vector<Vertice>::iterator &center, const  std::vector<Vertice>::iterator &point);
  
  /**
   * @brief Check if one given point of the polygon is an ear
   * 
   * @param vertice the candidate vertice
   * @return true iif vertice is an ear
   */
  bool isEar(const std::vector<Vertice>::iterator &vertice);
  
 public:
  /**
   * @brief A vertices constructor that takes a list of points and transforms them to vertices
   * @param points list of points
   */
  Vertices(const std::vector<geometry_msgs::Point> &points);
  
  /**
   * @brief Computes the triangularisation of the polygon using the ear-clipping method (destroy the vertices vector)
   * 
   * @param[out] triangles vector that stores the triangles (every triangle is three consecutive points such as 0,1,2 3,4,5 etc.)
   */
  void triangles(std::vector<geometry_msgs::Point> &triangles);
}; // Class Vertices


/**
 * @brief Computes the triangularisation of a polygon represented by a list of points, its vertices
 * 
 * @param points the vertices of the polygon
 * @return triangles vector that stores the triangularisation (every triangle is three consecutive points such as 0,1,2 3,4,5 etc.)
 */
inline std::vector<geometry_msgs::Point> triangularisation(std::vector<geometry_msgs::Point> &points) {
  std::vector<geometry_msgs::Point> triangles;
  if(points.size() <= 2) return triangles;
  Vertices vertices = Vertices(points);
  vertices.triangles(triangles);
  return triangles;
}

/** 
 * @brief Checks if the polygon's edges are stored clockwise
 *
 * @param polygon the edges of the polygon
 * @return true iif the polygon is stored clockwise
 */
inline bool isClockwise(const std::vector<custom_robot_msgs::Segment> &polygon) {
  if(polygon.size() < 3) return true;
  return ((polygon[1].p.x*polygon[2].p.y + polygon[0].p.x*polygon[1].p.y + polygon[0].p.y*polygon[2].p.x) - (polygon[0].p.y*polygon[1].p.x + polygon[1].p.y*polygon[2].p.x + polygon[0].p.x*polygon[2].p.y) > 0);
}

/** 
 * @brief Transforms a segment into a list of points
 *
 * @param seg a segment
 * @return the associated list of points
 */
inline std::vector<geometry_msgs::Point> fromSegmentToPoints(const custom_robot_msgs::Segment &segment) {
  std::vector<geometry_msgs::Point> points;
  points.push_back(segment.p);
  points.push_back(segment.q);
  return points;
}

/** 
 * @brief Transforms a list of segments into a list of points
 *
 * @param polygon a list of segments
 * @return the associated list of points
 */
inline std::vector<geometry_msgs::Point> fromSegmentToPoints(const std::vector<custom_robot_msgs::Segment> &polygon) {
  std::vector<geometry_msgs::Point> points;
  for(auto segment = polygon.begin(); segment != polygon.end(); segment++) {
    points.push_back(segment->p);
    points.push_back(segment->q);
  }
  return points;
}

/** 
 * @brief Extracts a polygon's vertices from its edges, and stores them clockwisely
 *
 * @param polygon the edges of the polygon
 * @return the stored vertices of the polygon
 */
std::vector<geometry_msgs::Point> getClockwisePolygon(const std::vector<custom_robot_msgs::Segment> &polygon); 

} // namespace polygon_helpers

#endif // GLOBAL_LIBRARY_H

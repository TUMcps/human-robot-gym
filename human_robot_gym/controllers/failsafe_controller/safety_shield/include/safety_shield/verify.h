// -*- lsst-c++ -*/
/**
 * @file verify_iso.h
 * @brief Defines the verify ISO class
 * @version 0.1
 * @copyright MIT License. Please see package.xml for further detail.
 */

#include <algorithm>
#include <vector>

#include <ros/ros.h>

#include "custom_robot_msgs/CapsuleArray.h"
#include "custom_robot_msgs/StartGoalCapsuleArray.h"
#include "custom_robot_msgs/PolycapsuleArray.h"
#include "custom_robot_msgs/BoolHeadered.h"
#include "global_library/global_library.h"

#ifndef VERIFY_H
#define VERIFY_H

namespace safety_shield {

/**
 * @brief Verifies if a given robot motion is safe with respect to a humans motion
 */
class Verify {
 public:
  /**
   * @brief A basic VerifyISO constructor
   */
  Verify() {}

  /**
   * @brief Check two capsules for collision
   * 
   * @param[in] cap1 Capsule 1
   * @param[in] cap2 Capsule 2
   * 
   * @returns true if capsules collide, false else
   */
  inline bool capsuleCollisionCheck(const custom_robot_msgs::Capsule& cap1, const custom_robot_msgs::Capsule& cap2) {
    double dist = segmentSegmentDist(cap1.segment, cap2.segment);
    return (dist <= cap1.radius + cap2.radius);
  }
  
  /**
   * @brief Calculate the distance between two arbitrary segments (can be spheres or real segments.)
   * 
   * @param[in] seg1 Segment 1
   * @param[in] seg2 Segment 2
   * 
   * @returns Minimal distance between the two segments
   */
  inline double segmentSegmentDist(const custom_robot_msgs::Segment& seg1, const custom_robot_msgs::Segment& seg2) {
    // First check if any capsule is a sphere
    bool is_sphere1 = (geometry_helpers::equal(seg1.p, seg1.q));
    bool is_sphere2 = (geometry_helpers::equal(seg2.p, seg2.q));
    // Sphere collision check
    if (is_sphere1 && is_sphere2) {
      return pointPointDistance(seg1.p, seg2.p);
    }
    // Sphere and capsule collision check
    else if (is_sphere1 && !is_sphere2) {
      return pointSegmentDistance(seg1.p, seg2);
    }
    // Sphere and capsule collision check
    else if (!is_sphere1 && is_sphere2) {
      return pointSegmentDistance(seg2.p, seg1);
    }
    // Capsule and capsule collision
    else {
      return segmentsDistance(seg1, seg2);
    }
  }
  
  /**
   * @brief Computes the distance between a point and a segment.
   * 
   * @param[in] p Point
   * @param[in] seg Segment
   * 
   * @returns Distance between point and segment
   */
  inline double pointSegmentDistance(const geometry_msgs::Point& p, const custom_robot_msgs::Segment& seg) {
    // https://de.mathworks.com/matlabcentral/answers/260593-distance-between-points-and-a-line-segment-in-3d
    // Vector from start to end of segment
    geometry_msgs::Point se = geometry_helpers::fromSegmentToVector(seg.q, seg.p);
    // Length of segment
    double dse = geometry_helpers::norm(se);
    // Distance from start point to point
    geometry_msgs::Point sp = geometry_helpers::fromSegmentToVector(p, seg.p);
    double dsp = geometry_helpers::norm(sp);
    // Distance from end point to point
    geometry_msgs::Point ep = geometry_helpers::fromSegmentToVector(p, seg.q);
    double dep = geometry_helpers::norm(ep);
    // Type 1: Point closest to end point
    if (sqrt(pow(dse,2) + pow(dep,2)) <= dsp) {
      return dep;
    }
    // Type 2: Point closest to start point
    if (sqrt(pow(dse,2) + pow(dsp,2)) <= dep) {
      return dsp;
    }
    // Type 3: Point in between start and end point 
    return (geometry_helpers::norm(geometry_helpers::cross(se, ep))/dse);
  }

  /**
   * @brief Computes the distance between two points
   * 
   * @param[in] p1 Point
   * @param[in] p2 Point
   * 
   * @returns Distance between point and point
   */
  inline double pointPointDistance(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) {
    return geometry_helpers::norm(geometry_helpers::fromSegmentToVector(p1, p2));
  }
  
  /**
   * @brief Test for ray edge cross
   * 
   * Subsfunction of the isIn method
   * return true iff the ray starting from p and parallel to the x axis intersects the edge
   *
   * @param p    the starting point of the ray
   * @param edge the edge
   * 
   * @returns (p, inf) intersect edge?
   */
  inline bool cross(const custom_robot_msgs::Segment& edge, const geometry_msgs::Point& p) {
    if ((p.y >= edge.p.y && p.y < edge.q.y) || (p.y >= edge.p.y && p.y < edge.q.y)) {
      double inter_x = (p.y - edge.p.y) * (edge.q.x - edge.p.x) / (edge.q.y - edge.p.y) + edge.p.x;
      return inter_x < p.x;
    }
    return false;
  }

  /**
   * @brief Checks if a given point is inside a given polygon
   *
   * @param p       the given point
   * @param polygon the given polygon
   * 
   * @returns p in polygon?
   */
  bool isIn(const geometry_msgs::Point& p, const std::vector<custom_robot_msgs::Segment>& polygon) {
    bool is_in = false;
    for (auto edge = polygon.begin(); edge != polygon.end(); edge++) {
      if (cross(*edge, p)) {
        is_in != is_in;
      }
    }
    return is_in;
  }
  
  /**
   * @brief Computes the square of the distance between two given segments
   * 
   * @param seg1 the first segment
   * @param seg2 the second segment
   * 
   * @returns dist(seg1,seg2)²
   */
  double segmentsDistance(const custom_robot_msgs::Segment &seg1, const custom_robot_msgs::Segment &seg2);

  /**
   * @brief Verify the robot motion againt the reachability analysis of the human in position, velocity, and acceleration
   * 
   * Pure virtual function.
   * 
   * @param[in] robot_capsules Reachable capsules of the robot
   * @param[in] human_reach_capsules_P Reachable capsules of the human according to position approach
   * @param[in] human_reach_capsules_V Reachable capsules of the human according to velocity approach
   * @param[in] human_reach_capsules_A Reachable capsules of the human according to acceleration approach
   * 
   * @returns Whether the robot movement is unsafe for the human
   */
  virtual bool verify_human_reach(const custom_robot_msgs::StartGoalCapsuleArray* robot_capsules, 
    const custom_robot_msgs::CapsuleArray* human_reach_capsules_P, 
    const custom_robot_msgs::CapsuleArray* human_reach_capsules_V, 
    const custom_robot_msgs::CapsuleArray* human_reach_capsules_A) = 0;
};
} // namespace safety_shield

#endif // VERIFY_H

// -*- lsst-c++ -*/
/**
 * @file verify_iso.h
 * @brief Defines the verify ISO class
 * @version 0.1
 * @copyright MIT License. Please see package.xml for further detail.
 */

#include <algorithm>
#include <vector>

#include "reach_lib.hpp"

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
  inline bool capsuleCollisionCheck(const reach_lib::Capsule& cap1, const reach_lib::Capsule& cap2) {
    return reach_lib::intersections::capsule_capsule_intersection(cap1, cap2);
  }
  
  /**
   * @brief Verify the robot motion against the reachable occupancy of the human in position, velocity, and acceleration
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
  virtual bool verify_human_reach(const std::vector<reach_lib::Capsule>* robot_capsules, 
    const reach_lib::ArticulatedPos* human_reach_capsules_P, 
    const reach_lib::ArticulatedVel* human_reach_capsules_V, 
    const reach_lib::ArticulatedAccel* human_reach_capsules_A) = 0;
};
} // namespace safety_shield

#endif // VERIFY_H

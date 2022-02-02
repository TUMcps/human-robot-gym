// -*- lsst-c++ -*/
/**
 * @file verify_iso.h
 * @brief Defines the verify ISO class
 * @version 0.1
 * @copyright MIT License. Please see package.xml for further detail.
 */

#include "safety_shield/verify.h"

#include <algorithm>
#include <vector>

#include "spdlog/spdlog.h" // https://github.com/gabime/spdlog

#include <reach_lib.hpp>

#ifndef VERIFY_ISO_H
#define VERIFY_ISO_H

namespace safety_shield {

/**
 * @brief Verifies if a given robot motion is safe with respect to a humans motion
 */
class VerifyISO : public Verify {
 public:
  /**
   * @brief A basic VerifyISO constructor
   */
  VerifyISO() : Verify() {}
  /**
   * @brief Check a set of robot capsules if they collide with a set of human capsules
   * 
   * @param[in] robot_capsules The robot capsules
   * @param[in] human_capsules The human occupancy capsules
   * 
   * @returns Whether a collision between any two capsules of the robot and human set occured
   */
  bool robotHumanCollision(const std::vector<reach_lib::Capsule>& robot_capsules, 
      const std::vector<reach_lib::Capsule>& human_capsules);

  /**
   * @brief Verify the robot motion againt the reachability analysis of the human in position, velocity, and acceleration
   * 
   * @param[in] robot_capsules Reachable capsules of the robot
   * @param[in] human_reach_capsules_P Reachable capsules of the human according to position approach
   * @param[in] human_reach_capsules_V Reachable capsules of the human according to velocity approach
   * @param[in] human_reach_capsules_A Reachable capsules of the human according to acceleration approach
   * 
   * @returns Whether the robot movement is unsafe for the human
   */
  bool verify_human_reach(const std::vector<reach_lib::Capsule>& robot_capsules, 
      reach_lib::ArticulatedPos& human_reach_capsules_P, 
      reach_lib::ArticulatedVel& human_reach_capsules_V, 
      reach_lib::ArticulatedAccel& human_reach_capsules_A);
};
} // namespace safety_shield

#endif // VERIFY_ISO_H

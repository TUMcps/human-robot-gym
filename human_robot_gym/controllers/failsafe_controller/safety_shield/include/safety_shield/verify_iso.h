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

#include <ros/ros.h>

#include "custom_robot_msgs/CapsuleArray.h"
#include "custom_robot_msgs/StartGoalCapsuleArray.h"
#include "custom_robot_msgs/PolycapsuleArray.h"
#include "custom_robot_msgs/BoolHeadered.h"
#include "global_library/global_library.h"

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
   * @param[in] human_capsules The human capsules
   * 
   * @returns Whether a collision between any two capsules of the robot and human set occured
   */
  bool robotHumanCollision(const custom_robot_msgs::CapsuleArray* robot_capsules, 
      const custom_robot_msgs::CapsuleArray* human_capsules);

  /**
   * @brief Verify if robot motion is safe
   * Reads the reachable occupancy of the robot and of the human in the associated buffer and checks (and sends using the pub_safe publisher) if there is an intersection between them 
   *
   * @param robot_ra the reachable occupancy of the robot (list of capsules)
   * @param human_ra the reachable occupancy of the human (list of polycapsules)
   * 
   * @returns Whether the robot movement is unsafe for the human
   */
  bool verify(const custom_robot_msgs::CapsuleArray* robot_ra, 
      const custom_robot_msgs::PolycapsuleArray* human_ra);

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
  bool verify_human_reach(const custom_robot_msgs::CapsuleArray* robot_capsules, 
      const custom_robot_msgs::CapsuleArray* human_reach_capsules_P, 
      const custom_robot_msgs::CapsuleArray* human_reach_capsules_V, 
      const custom_robot_msgs::CapsuleArray* human_reach_capsules_A);

  /**
   * @brief Verify the robot motion againt the reachability analysis of the human in position, velocity, and acceleration
   * 
   * Wrapper function of `verify_human_reach` to use the more general `StartGoalCapsuleArray` message
   * 
   * @param[in] robot_capsules Reachable capsules of the robot
   * @param[in] human_reach_capsules_P Reachable capsules of the human according to position approach
   * @param[in] human_reach_capsules_V Reachable capsules of the human according to velocity approach
   * @param[in] human_reach_capsules_A Reachable capsules of the human according to acceleration approach
   * 
   * @returns Whether the robot movement is unsafe for the human
   */
  bool verify_human_reach(const custom_robot_msgs::StartGoalCapsuleArray* robot_capsules, 
    const custom_robot_msgs::CapsuleArray* human_reach_capsules_P, 
    const custom_robot_msgs::CapsuleArray* human_reach_capsules_V, 
    const custom_robot_msgs::CapsuleArray* human_reach_capsules_A) {
      custom_robot_msgs::CapsuleArray robot_caps;
      robot_caps.header = robot_capsules->header;
      robot_caps.capsules = robot_capsules->capsules;
      return verify_human_reach(&robot_caps, human_reach_capsules_P, human_reach_capsules_V, human_reach_capsules_A);
  }
};
} // namespace safety_shield

#endif // VERIFY_ISO_H

// -*- lsst-c++ -*/
/**
 * @file advanced_verify_iso.h
 * @brief Defines the advanced verify ISO class
 * @version 0.1
 * @copyright MIT License. Please see package.xml for further detail.
 */

#include "safety_shield/verify.h"

#include <algorithm>
#include <vector>
#include <map>
#include <utility>
	
#include <ros/ros.h>

#include "custom_robot_msgs/StartGoalCapsuleArray.h"
#include "custom_robot_msgs/BoolHeadered.h"
#include "global_library/global_library.h"


#ifndef ADVANCED_VERFIY_ISO_H
#define ADVANCED_VERFIY_ISO_H

namespace safety_shield {

/**
 * @brief Verifies if a given robot motion is safe with respect to a humans motion
 * 
 * Additionally allows the robot to move away from the human even if the capsules would collide.
 */
class AdvancedVerifyISO : public Verify{

 private:

  /**
   * @brief Check if the goal segment is safer than the start segment
   * @param[in] start_segment The robot segment at the start of the motion
   * @param[in] goal_segment The robot segment at the goal of the motion
   * @param[in] human_segment The segment of the human capsule
   * @param[in] min_safety_dist The minimum difference between start-human and goal-human
   * @param[in] min_move_dist The minimum distance to the human to allow any moves
   * @returns True if the goal motion is safer than the start
   */
  inline bool startGoalSafetyCheck(const custom_robot_msgs::Segment& start_segment, 
      const custom_robot_msgs::Segment& goal_segment,
      const custom_robot_msgs::Segment& human_segment,
      double min_safety_dist,
      double min_move_dist) {
    double start_dist = segmentSegmentDist(start_segment, human_segment);
    double goal_dist = segmentSegmentDist(goal_segment, human_segment);
    // Goal distance should be larger than start distance
    return ((goal_dist - start_dist) > min_safety_dist && start_dist >= min_move_dist);
  }

  /**
   * @brief Check if two robot parts could create a harmful clamp
   * @param[in] start_segment_1 The robot segment (part 1) at the start of the motion
   * @param[in] goal_segment_1 The robot segment (part 1) at the goal of the motion
   * @param[in] start_segment_2 The robot segment (part 2) at the start of the motion
   * @param[in] goal_segment_2 The robot segment (part 2) at the goal of the motion
   * @param[in] far_away_distance Ignore parts that are far away from each other
   * @returns If a clamp can occur
   */
  inline bool clampCheck(const custom_robot_msgs::Segment& start_segment_1, 
      const custom_robot_msgs::Segment& goal_segment_1,
      const custom_robot_msgs::Segment& start_segment_2, 
      const custom_robot_msgs::Segment& goal_segment_2,
      double far_away_distance) {
    // Check for far away 
    double start_start_smallest_dist = segmentSegmentDist(start_segment_1, start_segment_2);
    if (start_start_smallest_dist > far_away_distance) {
      return false;
    }
    // Validate if closest point before motion is farther away than after motion
    double goal_goal_smallest_dist = segmentSegmentDist(goal_segment_1, goal_segment_2);
    if (goal_goal_smallest_dist < start_start_smallest_dist) {
      return true;
    } else {
      return false;
    }
  }

 public:
  /**
   * @brief Basic constructor
   */
  AdvancedVerifyISO() : Verify() {}

  /**
   * @brief Check a set of robot capsules if they collide with a set of human capsules and if the movement is safe.
   * 
   * @param[in] robot_capsules The robot capsules
   * @param[in] human_capsules The human capsules
   * 
   * @returns Whether the robot movement is unsafe for the human
   */
  bool robotHumanCollision(const custom_robot_msgs::StartGoalCapsuleArray* robot_capsules, const custom_robot_msgs::CapsuleArray* human_capsules);

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
  bool verify_human_reach(const custom_robot_msgs::StartGoalCapsuleArray* robot_capsules, 
    const custom_robot_msgs::CapsuleArray* human_reach_capsules_P, 
    const custom_robot_msgs::CapsuleArray* human_reach_capsules_V, 
    const custom_robot_msgs::CapsuleArray* human_reach_capsules_A);
};
} // namespace safety_shield

#endif // ADVANCED_VERFIY_ISO_H

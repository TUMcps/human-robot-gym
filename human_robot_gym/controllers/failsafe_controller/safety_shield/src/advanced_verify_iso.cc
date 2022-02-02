#include "safety_shield/advanced_verify_iso.h"

namespace safety_shield {


bool AdvancedVerifyISO::robotHumanCollision(const custom_robot_msgs::StartGoalCapsuleArray* robot_capsules, 
    const custom_robot_msgs::CapsuleArray* human_capsules) {
  // key: Robot capsule id, values: Human capsules that are in collision with this robot capsule
  std::map<int, std::vector<int>> robot_human_collisions;
  bool is_safe = true;
  // Check position capsules
  for (int i = 0; i < robot_capsules->capsules.size(); i++) {
    for(int j = 0; j < human_capsules->capsules.size(); j++) {
      // If there is a collision, safe it to map
      if (capsuleCollisionCheck(robot_capsules->capsules[i], human_capsules->capsules[j])) {
        if (robot_human_collisions.count(i) == 0) {
          robot_human_collisions.insert(std::pair<int, std::vector<int>> (i, std::vector<int>()));
        }
        robot_human_collisions.at(i).push_back(j);
        is_safe = false;
      }
    }
  }
  // If no collision occured, return safe
  if (is_safe) {
    return false;
  }
  // First check every collision if the movement would lead to a safer position
  for (auto& collision_pair : robot_human_collisions) {
    int i = collision_pair.first;
    for (int j : collision_pair.second) {
      bool movement_safe = startGoalSafetyCheck(robot_capsules->starts[i], robot_capsules->goals[i], human_capsules->capsules[j].segment, 0.0, 2*robot_capsules->capsules[i].radius);
      // If one movement would not lead to a safer state, return unsafe.
      if (!movement_safe) {
        return true;
      }
    }
  }
  // If all movements would be safe, check if there are any potential clamps
  // We assume this is not neccessary right now, since when every robot elements moves away from the potential human collision,
  // there cannot be any clamps.

  // return safe
  return false;
}


bool AdvancedVerifyISO::verify_human_reach(const custom_robot_msgs::StartGoalCapsuleArray* robot_capsules, 
    const custom_robot_msgs::CapsuleArray* human_reach_capsules_P, 
    const custom_robot_msgs::CapsuleArray* human_reach_capsules_V, 
    const custom_robot_msgs::CapsuleArray* human_reach_capsules_A) {
  try {
    // If no collision occured, we are safe and don't have to check the rest.
    if(!robotHumanCollision(robot_capsules, human_reach_capsules_P)) {
      return true;
    }
    // If no collision occured, we are safe and don't have to check the rest.
    if(!robotHumanCollision(robot_capsules, human_reach_capsules_V)) {
      return true;
    }
    // If no collision occured, we are safe and don't have to check the rest.
    if(!robotHumanCollision(robot_capsules, human_reach_capsules_A)) {
      return true;
    }
    // We are not safe and publish that.
    return false;
  } catch (const std::exception &exc) {
    ROS_ERROR_STREAM("Exception in AdvancedVerifyISO::verify_human_reach: " << exc.what());
    return false;
  }
}
} // namespace safety_shield
#include "safety_shield/verify_iso.h"

namespace safety_shield {


bool VerifyISO::robotHumanCollision(const std::vector<reach_lib::Capsule>& robot_capsules, 
      const std::vector<reach_lib::Capsule>& human_capsules) {
  // Check position capsules
  for(auto& human_capsule : human_capsules) {
    for (auto& robot_capsule : robot_capsules) {
      // If there is a collision, return true
      if (capsuleCollisionCheck(robot_capsule, human_capsule)) {
        return true;
      }
    }
  }
  return false;
}

bool VerifyISO::verify_human_reach(const std::vector<reach_lib::Capsule>& robot_capsules, 
      reach_lib::ArticulatedPos& human_reach_capsules_P, 
      reach_lib::ArticulatedVel& human_reach_capsules_V, 
      reach_lib::ArticulatedAccel& human_reach_capsules_A) {
  try {
    // If no collision occured, we are safe and don't have to check the rest.
    if(!robotHumanCollision(robot_capsules, reach_lib::get_capsules(human_reach_capsules_P))) {
      return true;
    }
    // If no collision occured, we are safe and don't have to check the rest.
    if(!robotHumanCollision(robot_capsules, reach_lib::get_capsules(human_reach_capsules_V))) {
      return true;
    }
    // If no collision occured, we are safe and don't have to check the rest.
    if(!robotHumanCollision(robot_capsules, reach_lib::get_capsules(human_reach_capsules_A))) {
      return true;
    }
    return false;
  } catch (const std::exception &exc) {
    spdlog::error("Exception in VerifyISO::verify_human_reach: {}", exc.what());
    return false;
  }
}
} // namespace safety_shield
#include "safety_shield/verify_iso.h"

namespace safety_shield {


bool VerifyISO::robotHumanCollision(const custom_robot_msgs::CapsuleArray* robot_capsules, 
    const custom_robot_msgs::CapsuleArray* human_capsules) {
  // Check position capsules
  for(auto& human_capsule : human_capsules->capsules) {
    for (auto& robot_capsule : robot_capsules->capsules) {
      // If there is a collision, return true
      if (capsuleCollisionCheck(robot_capsule, human_capsule)) {
        return true;
      }
    }
  }
  return false;
}


bool VerifyISO::verify(const custom_robot_msgs::CapsuleArray* robot_ra, 
    const custom_robot_msgs::PolycapsuleArray* human_ra) { 
  custom_robot_msgs::BoolHeaderedPtr is_safe(new custom_robot_msgs::BoolHeadered());
  // check if there is a risk of collision
  for (auto robot_capsule = robot_ra->capsules.begin(); robot_capsule != robot_ra->capsules.end(); robot_capsule++) {
    for(auto human_polycapsule = human_ra->polycapsules.begin(); human_polycapsule != human_ra->polycapsules.end(); human_polycapsule++) {
      if (isIn(robot_capsule->segment.p, human_polycapsule->polygon)) {
        return false;
      }
      for(auto segment = human_polycapsule->polygon.begin(); segment != human_polycapsule->polygon.end(); segment++) {
        double distance = segmentsDistance(robot_capsule->segment, *segment);
        if ((distance - robot_capsule->radius - human_polycapsule->radius) <= 0) {
          return false;
        }
      }
    }
  }
  return true;
}


bool VerifyISO::verify_human_reach(const custom_robot_msgs::CapsuleArray* robot_capsules, 
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
    return false;
  } catch (const std::exception &exc) {
    ROS_ERROR_STREAM("Exception in VerifyISO::verify_human_reach: " << exc.what());
    return false;
  }
}
} // namespace safety_shield
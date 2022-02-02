#include "safety_shield/safety_shield.h"


namespace safety_shield {

SafetyShield::SafetyShield() {
  
}


SafetyShield::~SafetyShield() {

}

void SafetyShield::step(const ros::Time& cycle_begin_time) {
  std::this_thread::sleep_for(std::chrono::milliseconds(1));
}

} // namespace safety_shield
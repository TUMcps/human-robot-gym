#include <string>
#include <vector>

#include "safety_shield/safety_shield.h"
#include "safety_shield/human_reach.h"
#include "safety_shield/robot_reach.h"
#include "safety_shield/verify.h"
#include "safety_shield/long_term_traj.h"

int main () {
    bool activate_shield = true;
    double sample_time = 0.001; 
    std::string trajectory_config_file = std::string("../../safety_shield/config/trajectory_parameters_modrob1.yaml");
    std::string robot_config_file = std::string("../../safety_shield/config/robot_parameters_modrob1.yaml");
    std::string mocap_config_file = std::string("../../safety_shield/config/cmu_mocap_no_hand.yaml");
    double init_x = 0.0;
    double init_y = 0.0;
    double init_z = 0.0;
    double init_roll = 0.0;
    double init_pitch = 0.0;
    double init_yaw = 0.0;

    safety_shield::SafetyShield shield = safety_shield::SafetyShield(activate_shield,
      sample_time, 
      trajectory_config_file,
      robot_config_file,
      mocap_config_file,
      init_x, 
      init_y, 
      init_z, 
      init_roll, 
      init_pitch, 
      init_yaw);
    return 0;
}
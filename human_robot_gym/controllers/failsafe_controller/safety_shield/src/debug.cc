#include <string>
#include <vector>
#include <chrono>
#include <type_traits>

#include "SaRA/reach_lib.hpp"

#include "safety_shield/safety_shield.h"
#include "safety_shield/human_reach.h"
#include "safety_shield/robot_reach.h"
#include "safety_shield/verify.h"
#include "safety_shield/long_term_traj.h"
#include "safety_shield/motion.h"

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

    // Dummy human measurement
    std::vector<reach_lib::Point> dummy_human_meas(21);
    for (int i=0; i<21; i++) {
      dummy_human_meas[i] = reach_lib::Point(10.0, 10.0, 0.0);
    }

    //auto start_time = std::chrono::system_clock::now();
    //double t = std::chrono::duration<double>(std::chrono::system_clock::now()-start_time).count();
    double t = 0.0;
    for (int i=0; i<10000; i++) {
      t += 0.001;
      shield.humanMeasurement(dummy_human_meas, t);
      t += 0.003;
      if (i % 100 == 0) {
        std::vector<double> motion_vec{0.2*t, 0.0, 0.0, 0.0, 0.0, std::min(t, 3.1)};
        safety_shield::Motion goal_motion(t, motion_vec);
        shield.newLongTermTrajectory(goal_motion);
      }
      shield.step(t);
    }
    return 0;
}
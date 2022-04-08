#include "safety_shield/safety_shield.h"


namespace safety_shield {

SafetyShield::SafetyShield():
    max_s_stop_(0),
    v_max_allowed_({0, 0, 0}),
    a_max_allowed_({0, 0, 0}),
    j_max_allowed_({0, 0, 0}),
    a_max_ltt_({0, 0, 0}),
    j_max_ltt_({0, 0, 0})
    {
      spdlog::info("Safety shield created.");
    }

SafetyShield::SafetyShield(bool activate_shield,
      int nb_joints, 
      double sample_time, 
      double max_s_stop, 
      const std::vector<double> &v_max_allowed, 
      const std::vector<double> &a_max_allowed, 
      const std::vector<double> &j_max_allowed, 
      const std::vector<double> &a_max_path, 
      const std::vector<double> &j_max_path, 
      const LongTermTraj &long_term_trajectory, 
      RobotReach* robot_reach,
      HumanReach* human_reach,
      Verify* verify):
  activate_shield_(activate_shield),
  nb_joints_(nb_joints),
  max_s_stop_(max_s_stop),
  v_max_allowed_(v_max_allowed),
  a_max_allowed_(a_max_allowed),
  j_max_allowed_(j_max_allowed), 
  a_max_ltt_(a_max_path), 
  j_max_ltt_(j_max_path),
  sample_time_(sample_time),
  path_s_(0),
  path_s_discrete_(0),
  long_term_trajectory_(long_term_trajectory),
  robot_reach_(robot_reach),
  human_reach_(human_reach),
  verify_(verify)
{
  sliding_window_k_ = (int) std::floor(max_s_stop_/sample_time_);
  std::vector<double> prev_dq;
  for(int i = 0; i < 6; i++) {
    prev_dq.push_back(0.0);
    alpha_i_.push_back(1.0);
  }
  alpha_i_.push_back(1.0);

  is_safe_ = !activate_shield_;
  computesPotentialTrajectory(is_safe_, prev_dq);
  next_motion_ = determineNextMotion(is_safe_);
  spdlog::info("Safety shield created.");
}

SafetyShield::SafetyShield(bool activate_shield,
      double sample_time,
      std::string trajectory_config_file,
      std::string robot_config_file,
      std::string mocap_config_file,
      double init_x, 
      double init_y, 
      double init_z, 
      double init_roll, 
      double init_pitch, 
      double init_yaw):
    activate_shield_(activate_shield),
    sample_time_(sample_time),
    path_s_(0),
    path_s_discrete_(0)
  {
    ///////////// Build robot reach
    YAML::Node robot_config = YAML::LoadFile(robot_config_file);
    std::string robot_name = robot_config["robot_name"].as<std::string>();
    nb_joints_ = robot_config["nb_joints"].as<int>();
    std::vector<double> transformation_matrices = robot_config["transformation_matrices"].as<std::vector<double>>();
    std::vector<double> enclosures = robot_config["enclosures"].as<std::vector<double>>();
    double secure_radius = robot_config["secure_radius"].as<double>();
    robot_reach_ = new RobotReach(transformation_matrices, 
      nb_joints_, 
      enclosures, 
      init_x, init_y, init_z, 
      init_roll, init_pitch, init_yaw,
      secure_radius);
    ////////////// Setting trajectory variables
    YAML::Node trajectory_config = YAML::LoadFile(trajectory_config_file);
    max_s_stop_ = trajectory_config["max_s_stop"].as<double>();
    q_min_allowed_ = trajectory_config["q_min_allowed"].as<std::vector<double>>();
    q_max_allowed_ = trajectory_config["q_max_allowed"].as<std::vector<double>>();
    v_max_allowed_ = trajectory_config["v_max_allowed"].as<std::vector<double>>();
    a_max_allowed_ = trajectory_config["a_max_allowed"].as<std::vector<double>>();
    j_max_allowed_ = trajectory_config["j_max_allowed"].as<std::vector<double>>();
    a_max_ltt_ = trajectory_config["a_max_ltt"].as<std::vector<double>>();
    j_max_ltt_ = trajectory_config["j_max_ltt"].as<std::vector<double>>();
    std::vector<std::vector<double>> q_vals(nb_joints_);
    for (int i=1; i <= nb_joints_; i++){
        std::string qi = "q" + std::to_string(i);
        q_vals[i-1] = trajectory_config[qi].as<std::vector<double>>();
    }
    // store the long term trajectory
    std::vector<Motion> long_term_traj;
    for(int i = 0; i < q_vals[0].size(); i++){
        std::vector<double> angles(nb_joints_);
        for(int j=0; j < nb_joints_; j++){
            angles[j] = q_vals[j][i];
        }
        long_term_traj.push_back(Motion(i*sample_time_, angles));
    }
    long_term_trajectory_ = LongTermTraj(long_term_traj);
    //////////// Build human reach
    YAML::Node human_config = YAML::LoadFile(mocap_config_file);
    double measurement_error_pos = human_config["measurement_error_pos"].as<double>();
    double measurement_error_vel = human_config["measurement_error_vel"].as<double>();
    double delay = human_config["delay"].as<double>();

    std::vector<std::string> joint_name_vec = human_config["joint_names"].as<std::vector<std::string>>();
    std::map<std::string, int> joint_names;
    for(std::size_t i = 0; i < joint_name_vec.size(); ++i) {
        joint_names[joint_name_vec[i]] = i;
    }

    std::vector<double> joint_v_max = human_config["joint_v_max"].as<std::vector<double>>();
    std::vector<double> joint_a_max = human_config["joint_a_max"].as<std::vector<double>>();
    // Build bodies
    const YAML::Node& bodies = human_config["bodies"];
    std::map<std::string, reach_lib::jointPair> body_link_joints;
    std::map<std::string, double> thickness;
    for (YAML::const_iterator it = bodies.begin(); it != bodies.end(); ++it) {
      const YAML::Node& body = *it;
      body_link_joints[body["name"].as<std::string>()] = reach_lib::jointPair(joint_names[body["proximal"].as<std::string>()], joint_names[body["distal"].as<std::string>()]);
      thickness[body["name"].as<std::string>()] = body["thickness"].as<double>(); 
    }
    // Build extremities
    const YAML::Node& extremities = human_config["extremities"];
    std::vector<std::string> extremity_base_names;
    std::vector<std::string> extremity_end_names; 
    std::vector<double> extremity_length;
    for (YAML::const_iterator it = extremities.begin(); it != extremities.end(); ++it) {
      const YAML::Node& extremity = *it;
      extremity_base_names.push_back(extremity["base"].as<std::string>());
      extremity_end_names.push_back(extremity["end"].as<std::string>());
      extremity_length.push_back(extremity["length"].as<double>());
    }
    human_reach_ = new HumanReach(joint_names.size(), 
      body_link_joints, 
      thickness, 
      joint_v_max, 
      joint_a_max,
      extremity_base_names, 
      extremity_end_names, 
      extremity_length,
      measurement_error_pos, 
      measurement_error_vel, 
      delay);
    ///////////// Build verifier
    verify_ = new safety_shield::VerifyISO();
    /////////// Other settings
    sliding_window_k_ = (int) std::floor(max_s_stop_/sample_time_);
    std::vector<double> prev_dq;
    for(int i = 0; i < 6; i++) {
        prev_dq.push_back(0.0);
        alpha_i_.push_back(1.0);
    }
    alpha_i_.push_back(1.0);
    is_safe_ = !activate_shield_;
    computesPotentialTrajectory(is_safe_, prev_dq);
    next_motion_ = determineNextMotion(is_safe_);
    spdlog::info("Safety shield created.");
  }

bool SafetyShield::planSafetyShield(double pos, double vel, double acc, double ve, double a_max, double j_max, Path &path) {
  if (a_max < 0 || fabs(acc) > a_max) {
    return false;
  }
  try {
    path.setCurrent(false);
    path.setPosition(pos);
    path.setVelocity(vel);
    path.setAcceleration(acc);
    double epsilon = 1e-6;

    // If current velocity is close to goal and acceleration is approx zero -> Do nothing
    if (fabs(vel - ve) < epsilon && fabs(acc) < epsilon) {
      std::array<double,6> new_phases = { sample_time_, sample_time_, sample_time_, 0.0, 0.0, 0.0};
      path.setPhases(new_phases);
      return true;
    }
    // Time to reach zero acceleration with maximum jerk
    double t_to_a_0 = roundToTimestep(abs(acc)/j_max);
    // Velocity at zero acceleration
    double v_at_a_0 = vel + acc*t_to_a_0/2;
    
    // If the velocity at zero acceleration is the goal velocity, use t_to_a_0 and max jerk and return 
    if (fabs(v_at_a_0 - ve) < epsilon){
      std::array<double,6> new_phases = { t_to_a_0, t_to_a_0, t_to_a_0, -acc/t_to_a_0, 0, 0 };
      path.setPhases(new_phases);
      return true;
    }
    
    double a_peak,t01,t12,t23,v01,v12,v23,correct_int;
    // If we are accelerating
    if (acc > 0) {
      // Reducing the acceleration to zero with max jerk would lead to a velocity larger than target
      if (v_at_a_0 > ve + epsilon) {
        if (vel - (a_max*a_max + acc*acc/2)/j_max > ve) {
          a_peak = -a_max;
          t01 = roundToTimestep((acc - a_peak)/j_max);
          t23 = roundToTimestep(-a_peak/j_max);
          v01 = (acc + a_peak)*t01/2;
          v23 = a_peak*t23/2;
          t12 = roundToTimestep((ve - vel - v01 - v23)/a_peak);
          v12 = a_peak*t12;
        }
        else {
          a_peak = -sqrt(fabs((vel - ve)*j_max + acc*acc/2));
          t01 = roundToTimestep((acc - a_peak)/j_max);
          t23 = roundToTimestep(-a_peak/j_max);
          v01 = (acc + a_peak)*t01/2;
          v23 = a_peak*t23/2;
          t12 = 0;
          v12 = 0;
        }
      }
      else {
        if (vel + (a_max*a_max - acc*acc/2)/j_max < ve) {
          a_peak = a_max;
          t01 = roundToTimestep((a_peak - acc)/j_max);
          t23 = roundToTimestep(a_peak/j_max);
          v01 = (acc + a_peak)*t01/2;
          v23 = a_peak*t23/2;
          t12 = roundToTimestep((ve - vel - v01 - v23)/a_peak);
          v12 = a_peak*t12;
        }
        else {
          a_peak = sqrt(fabs((ve - vel)*j_max + acc*acc/2));
          t01 = roundToTimestep((a_peak - acc)/j_max);
          t23 = roundToTimestep(a_peak/j_max);
          v01 = (acc + a_peak)*t01/2;
          v23 = a_peak*t23/2;
          t12 = 0;
          v12 = 0;
        }
      }
    }
    // If we are decelerating (a <= 0)
    else {
      if (v_at_a_0 > ve + epsilon) {
        if (vel - (a_max*a_max - acc*acc/2)/j_max > ve) {
          a_peak = -a_max;
          t01 = roundToTimestep((acc - a_peak)/j_max);
          t23 = roundToTimestep(-a_peak/j_max);
          v01 = (acc + a_peak)*t01/2;
          v23 = a_peak*t23/2;
          t12 = roundToTimestep((ve - vel - v01 - v23)/(a_peak));
          v12 = a_peak*t12;
        }
        else {
          a_peak = -sqrt(fabs((vel - ve)*j_max + acc*acc/2));
          t01 = roundToTimestep((acc - a_peak)/j_max);
          t23 = roundToTimestep(-a_peak/j_max);
          v01 = (acc + a_peak)*t01/2;
          v23 = a_peak*t23/2;
          t12 = 0;
          v12 = 0;
        }
      }
      else {
        if (vel + (a_max*a_max + acc*acc/2)/j_max < ve) {
          a_peak = a_max;
          t01 = roundToTimestep((a_peak - acc)/j_max);
          t23 = roundToTimestep(a_peak/j_max);
          v01 = (acc + a_peak)*t01/2;
          v23 = a_peak*t23/2;
          t12 = roundToTimestep((ve - vel - v01 - v23)/a_peak);
          v12 = a_peak*t12;
        }
        else {
          a_peak = sqrt(fabs((ve - vel)*j_max + acc*acc/2));
          t01 = roundToTimestep((a_peak - acc)/j_max);
          t23 = roundToTimestep(a_peak/j_max);
          v01 = (acc + a_peak)*t01/2;
          v23 = a_peak*t23/2;
          t12 = 0;
          v12 = 0;
        }
      }
    }
    correct_int = ve - vel - v01 - v12 - v23;
    a_peak += correct_int/((t01 + t23)/2 + t12 + epsilon);
    double j01;
    if (t01 >= epsilon) {
      j01 = (a_peak - acc)/(t01);
    } else {
      j01 = 0;
    }
    double j12 = 0;
    double j23;
    if (t23 >= epsilon) {
      j23 = -a_peak/(t23);
    } else {
      j23 = 0;
    }
    if (t01 < 0 || t12 < 0 || t23 < 0) {
      spdlog::debug("planSafetyShield calculated time negative. t01 = {}, t12 = {}, t23 = {}", t01, t12, t23);
      return false;
    }
    std::array<double,6> new_phases = { t01, t01+t12, t01+t12+t23, j01, j12, j23 };
    path.setPhases(new_phases);
    return true;
  } catch (const std::exception &exc) {
    spdlog::error("Exception in SafetyShield::planSafetyShield: {}", exc.what());
    return false;
  }
}

void SafetyShield::calculateMaxAccJerk(const std::vector<double> &prev_speed, const std::vector<double>& a_max_part, const std::vector<double>& j_max_part, double& a_max_manoeuvre, double& j_max_manoeuvre) {
  double new_c, new_d;

  double denom = std::abs(prev_speed[0]) + std::abs(a_max_part[0]) * max_s_stop_ + 1E-9;
  double min_c = (a_max_allowed_[0] - std::abs(a_max_part[0])) / denom;
  double min_d = (j_max_allowed_[0] - 3*std::abs(a_max_part[0])*a_max_allowed_[0] - std::abs(j_max_part[0])) / denom;
  spdlog::debug("calculateMaxAccJerk new_c = {}, new_d = {}, a_max_part[{}] = {}, j_max_part[{}] = {}", min_c, min_d, 0, std::abs(a_max_part[0]), 0, std::abs(j_max_part[0]));
  for (int i = 1; i < a_max_allowed_.size(); i++) {
    denom = std::abs(prev_speed[i]) + std::abs(a_max_part[i]) * max_s_stop_;
    new_c = (a_max_allowed_[i] - std::abs(a_max_part[i])) / denom;
    min_c = (new_c < min_c) ? new_c : min_c;
  
    new_d = (j_max_allowed_[i] - 3*std::abs(a_max_part[i])*new_c - std::abs(j_max_part[i])) / denom;
    min_d = (new_d < min_d) ? new_d : min_d;
    spdlog::debug("calculateMaxAccJerk new_c = {}, new_d = {}, a_max_part[{}] = {}, j_max_part[{}] = {}", new_c, new_d, i, std::abs(a_max_part[i]), i, std::abs(j_max_part[i]));
  }

  a_max_manoeuvre = (min_c < 0) ? 0 : min_c;
  j_max_manoeuvre = (min_d < 0) ? 0 : min_d;
  
  spdlog::debug("calculateMaxAccJerk denom = {}, a_max_manoeuvre = {}, j_max_manoeuvre = {}", denom, a_max_manoeuvre, j_max_manoeuvre);
}

Motion SafetyShield::computesPotentialTrajectory(bool v, const std::vector<double> &prev_speed)
{
  try {
    // s_int indicates the index of the entire traveled way
    while (path_s_ >= (path_s_discrete_+1)*sample_time_) {
      long_term_trajectory_.increasePosition();
      if (new_ltt_) {
        new_long_term_trajectory_.increasePosition();
      }
      path_s_discrete_++;
    }
    //If verified safe, take the recovery path, otherwise, take the failsafe path
    if (v && recovery_path_correct_) {
      recovery_path_.setCurrent(true);  
      //discard old FailsafePath and replace with new one
      failsafe_path_ = failsafe_path_2_;
      //repair path already incremented
    }
    else {
      failsafe_path_.setCurrent(true);
      //discard RepairPath
      recovery_path_.setCurrent(false);
      failsafe_path_.increment(sample_time_);
    }
    //find maximum acceleration and jerk authorised
    double a_max_manoeuvre, j_max_manoeuvre;
    // One could use new_long_term_trajectory_.getMaxAccelerationWindow(path_s_discrete_) instead of a_max_ltt but there are many bugs to be solved before.
    if (!new_ltt_) {
      calculateMaxAccJerk(prev_speed, a_max_ltt_, j_max_ltt_,a_max_manoeuvre, j_max_manoeuvre);
    } else {
      calculateMaxAccJerk(prev_speed, a_max_ltt_, j_max_ltt_,a_max_manoeuvre, j_max_manoeuvre);
    }
    
    //Desired movement, one timestep
    // if not already on the repair path, plan a repair path
    if (!recovery_path_.isCurrent()) {
      // plan repair path and replace
      recovery_path_correct_ = planSafetyShield(failsafe_path_.getPosition(), failsafe_path_.getVelocity(), 
          failsafe_path_.getAcceleration(), 1, a_max_manoeuvre, j_max_manoeuvre, recovery_path_);
    }
    // Only plan new failsafe trajectory if the recovery path planning was successful.
    if (recovery_path_correct_) {
      //advance one step on repair path
      recovery_path_.increment(sample_time_);

      // plan new failsafe path for STP
      bool failsafe_2_planning_success = planSafetyShield(recovery_path_.getPosition(), recovery_path_.getVelocity(), recovery_path_.getAcceleration(), 0, a_max_manoeuvre, j_max_manoeuvre, failsafe_path_2_);
      // Check the validity of the planned path
      if (!failsafe_2_planning_success || recovery_path_.getPosition() < failsafe_path_.getPosition()) {
        recovery_path_correct_ = false;
      }
    }
    // If all planning was correct, use new failsafe path with single recovery step
    if (recovery_path_correct_) {
      potential_path_ = failsafe_path_2_;
    }
    else {
      // If planning failed, use previous failsafe path
      potential_path_ = failsafe_path_;
    }

    //// Calculate start and goal pos of intended motion
    // Calculate start
    // Fill potential buffer with position and velocity from last failsafe path. This value is not really used.
    double s_d = failsafe_path_.getPosition();
    double ds_d = failsafe_path_.getVelocity();
    double dds_d = failsafe_path_.getAcceleration();
    // Always interpolate from current long term buffer
    Motion start_motion = interpolateFromTrajectory(s_d, ds_d, dds_d, long_term_trajectory_);
    // Calculate goal
    potential_path_.getFinalMotion(s_d, ds_d, dds_d);
    Motion goal_motion;
    if (new_ltt_) {
      goal_motion = interpolateFromTrajectory(s_d, ds_d, dds_d, new_long_term_trajectory_);
    } else {
      goal_motion = interpolateFromTrajectory(s_d, ds_d, dds_d, long_term_trajectory_);
    } 
    goal_motion.setTime(potential_path_.getPhase(3));
    //s_g_motion.header.stamp = cycle_begin_time_;
    //s_g_motion.start_motion = start_motion;
    //s_g_motion.goal_motion = goal_motion;
    //s_g_motion.duration = potential_path_.getPhase(3);
    return goal_motion;
  } catch (const std::exception &exc) {
    spdlog::error("Exception in SafetyShield::computesPotentialTrajectory: {}", exc.what());
    throw exc;
  }
}

bool SafetyShield::checkMotionForJointLimits(Motion& motion) {
  for (int i = 0; i < nb_joints_; i++) {
    if (motion.getAngle()[i] < q_min_allowed_[i] || motion.getAngle()[i] > q_max_allowed_[i]) {
      return false;
    }
  }
  return true;
}


Motion SafetyShield::determineNextMotion(bool is_safe) {
  Motion next_motion;
  double s_d, ds_d, dds_d;
  if (is_safe) {
    // Fill potential buffer with position and velocity from recovery path
    if (recovery_path_.getPosition() >= failsafe_path_.getPosition()) {
      s_d = recovery_path_.getPosition();
      ds_d = recovery_path_.getVelocity();
      dds_d = recovery_path_.getAcceleration();
    }
    else {
      potential_path_.increment(sample_time_);
      s_d = potential_path_.getPosition();
      ds_d = potential_path_.getVelocity();
      dds_d = potential_path_.getAcceleration();
    }

    // Interpolate from new long term buffer
    if (new_ltt_) {
      next_motion = interpolateFromTrajectory(s_d, ds_d, dds_d, new_long_term_trajectory_);
    } else {
      next_motion = interpolateFromTrajectory(s_d, ds_d, dds_d, long_term_trajectory_);
    }
    // Set potential path as new verified safe path
    safe_path_ = potential_path_;
  } else {
    // interpolate from old safe path
    safe_path_.increment(sample_time_);
    s_d = safe_path_.getPosition();
    ds_d = safe_path_.getVelocity();
    dds_d = safe_path_.getAcceleration();
    next_motion = interpolateFromTrajectory(s_d, ds_d, dds_d, long_term_trajectory_);
  }
  /// !!! Set s to the new path position !!!
  path_s_ = s_d;
  // Return the calculated next motion
  return next_motion;
}

Motion SafetyShield::interpolateFromTrajectory(double s, double ds, double dds, const LongTermTraj& trajectory) const {
  try {
    // Example: s=2.465, sample_time = 0.004 --> ind = 616.25
    assert(sample_time_ != 0);
    double ind = s/sample_time_;
    double intpart;
    // Example: intpart = 616.0, ind_mod = 0.25
    double ind_mod = modf(ind, &intpart);
    // floor(s/sample_time) + 1 ==> lower index
    int ind1 = static_cast<int>(intpart);
    // ceil(s/sample_time) + 1 ==> upper index
    int ind2 = static_cast<int>(ceil(ind));
    std::vector<double> q1 = trajectory.getNextMotionAtIndex(ind1).getAngle();
    std::vector<double> q2 = trajectory.getNextMotionAtIndex(ind2).getAngle();
    std::vector<double> q;
    std::vector<double> dq;
    std::vector<double> ddq;
    std::vector<double> dddq = trajectory.getNextMotionAtIndex(ind1).getJerk();

    assert(q1.size() == nb_joints_);
    assert(q2.size() == nb_joints_);
    for (int i = 0 ; i < nb_joints_; i++) {
      // Linearly interpolate between lower and upper index of position
      double q_clamped = q1[i] + ind_mod * (q2[i] - q1[i]);
      q.push_back(q_clamped);
      // Calculate LTT velocity
      double v_max_int = (q2[i] - q1[i])/sample_time_;
      double v_int = v_max_int * ds;
      if (std::abs(v_int) > v_max_allowed_[i]) {
        v_int = std::clamp(v_int, -v_max_allowed_[i], v_max_allowed_[i]);
      }
      dq.push_back(v_int);
      // Calculate Acceleration
      double a_int = v_max_int*dds;
      if (std::abs(a_int) > a_max_allowed_[i]) {
        a_int = std::clamp(a_int, -a_max_allowed_[i], a_max_allowed_[i]);
      }
      ddq.push_back(a_int);
    }
    return Motion(0.0, q, dq, ddq, dddq, s);
  } catch (const std::exception &exc) {
    spdlog::error("Exception in SafetyShield::interpolateFromTrajectory: {}", exc.what());
    return Motion();
  }

}

Motion SafetyShield::step(double cycle_begin_time) {
  cycle_begin_time_ = cycle_begin_time;
  try {
    // If the new LTT was processed at least once and is labeled safe, replace old LTT with new one.
    if (new_ltt_ && new_ltt_processed_ && is_safe_) {
      long_term_trajectory_ = new_long_term_trajectory_;
      new_ltt_ = false;
      new_goal_ = false;
    }
    // Get current motion
    Motion current_motion = getCurrentMotion();
    // Check if there is a new goal motion
    if (new_goal_) {
      // Check if current motion has acceleration and jerk values that lie in the plannable ranges
      bool is_plannable = checkCurrentMotionForReplanning(current_motion);
      if (is_plannable) {
        // Check if the starting position of the last replanning was very close to the current position
        bool last_close = true;
        if (new_ltt_ == true) {
          for (int i = 0; i < current_motion.getAngle().size(); i++) {
            if (std::abs(current_motion.getAngle()[i] - last_replan_start_motion_.getAngle()[i]) > 0.01) {
              last_close = false;
              break;
            }
          }
        } else {
          last_close = false;
        }
        // Only replan if the current joint position is different from the last.
        if (!last_close) {
          new_long_term_trajectory_ = calculateLongTermTrajectory(current_motion.getAngle(), current_motion.getVelocity(), 
            current_motion.getAcceleration(), new_goal_motion_.getAngle(), new_goal_motion_.getVelocity());
          last_replan_start_motion_ = current_motion;
        }
        new_ltt_ = true;
        new_ltt_processed_ = false;
      } else {
        new_ltt_ = false;
      }
    }
    // If there is a new long term trajectory (LTT), always override is_safe with false.
    if (new_ltt_ && !new_ltt_processed_) {
      is_safe_ = false;
    }
    // Compute a new potential trajectory
    Motion goal_motion = computesPotentialTrajectory(is_safe_, next_motion_.getVelocity());
    if (activate_shield_) {
      // Check motion for joint limits
      if (!checkMotionForJointLimits(goal_motion)) {
        is_safe_ = false;
      } else {
        // Compute the robot reachable set for the potential trajectory
        robot_capsules_ = robot_reach_->reach(current_motion, goal_motion, (goal_motion.getS()-current_motion.getS()), alpha_i_);
        // Compute the human reachable sets for the potential trajectory
        // humanReachabilityAnalysis(t_command, t_brake)
        human_reach_->humanReachabilityAnalysis(cycle_begin_time_, goal_motion.getTime());
        human_capsules_ = human_reach_->getAllCapsules();
        // Verify if the robot and human reachable sets are collision free
        is_safe_ = verify_->verify_human_reach(robot_capsules_, human_capsules_);
      }
    } else {
      is_safe_ = true;
    }
    // Select the next motion based on the verified safety
    next_motion_ = determineNextMotion(is_safe_);
    next_motion_.setTime(cycle_begin_time);
    new_ltt_processed_ = true;
    return next_motion_;
  } catch (const std::exception &exc) {
    spdlog::error("Exception in SafetyShield::getNextCycle: {}", exc.what());
  }
}

Motion SafetyShield::getCurrentMotion() {
  Motion current_pos;
  if (!recovery_path_.isCurrent()) {
    current_pos = interpolateFromTrajectory(failsafe_path_.getPosition(), failsafe_path_.getVelocity(), failsafe_path_.getAcceleration(), long_term_trajectory_);
  } else {
    current_pos = interpolateFromTrajectory(recovery_path_.getPosition(), recovery_path_.getVelocity(), recovery_path_.getAcceleration(), long_term_trajectory_); 
  }
  return current_pos;
}

bool SafetyShield::checkCurrentMotionForReplanning(Motion& current_motion) {
  for (int i = 0; i < nb_joints_; i++) {
    if (std::abs(current_motion.getAcceleration()[i]) > a_max_ltt_[i]) {
      return false;
    }
  }
  return true;
}

void SafetyShield::newLongTermTrajectory(Motion& goal_motion) {
  try { 
    std::vector<double> motion_q;
    motion_q.reserve(nb_joints_); 
    std::vector<double> motion_dq;
    motion_dq.reserve(nb_joints_); 
    for (int i = 0; i < nb_joints_; i++) {
      // TODO: replace with config max min values
      motion_q.push_back(std::clamp(goal_motion.getAngle()[i], q_min_allowed_[i], q_max_allowed_[i]));
      motion_dq.push_back(std::clamp(goal_motion.getVelocity()[i], -v_max_allowed_[i], v_max_allowed_[i]));
    }
    new_goal_motion_ = Motion(cycle_begin_time_, motion_q, motion_dq);
    new_goal_ = true;
    new_ltt_ = false;
    new_ltt_processed_ = false;
    std::vector<double> dummy_q;
    for (int i = 0; i < nb_joints_; i++) {
      // fill with high values to guarantee this is not close to current joint position
      dummy_q.push_back(-1234567);
    }
    last_replan_start_motion_ = Motion(cycle_begin_time_, dummy_q);
  } catch (const std::exception &exc) {
    spdlog::error("Exception in SafetyShield::newLongTermTrajectory: {}", exc.what());
  }
}


LongTermTraj SafetyShield::calculateLongTermTrajectory(const std::vector<double>& start_q, 
    const std::vector<double> start_dq, 
    const std::vector<double> start_ddq,
    const std::vector<double>& goal_q, 
    const std::vector<double> goal_dq) {
  // 0 = Not finished, 1 = finished, <0 = Error  
  int	ResultValue	=	0;
  // Creating all relevant objects of the Type IV Reflexxes Motion Library	
  reflexxes_RML_	=	new ReflexxesAPI(nb_joints_,	sample_time_);
  reflexxes_IP_	=	new RMLPositionInputParameters(nb_joints_);
  reflexxes_OP_	=	new RMLPositionOutputParameters(nb_joints_);

  for (int i = 0; i < nb_joints_; i++) {
    assert(start_dq[i] <= v_max_allowed_[i]);
    assert(start_ddq[i] <= a_max_ltt_[i]);
    ////ROS_INFO_STREAM("Joint " << i);
    reflexxes_IP_->CurrentPositionVector->VecData[i] = start_q[i];
    ////ROS_INFO_STREAM("CurrentPositionVector " << IP->CurrentPositionVector->VecData[i]);
    reflexxes_IP_->CurrentVelocityVector->VecData[i] = start_dq[i];
    ////ROS_INFO_STREAM("CurrentVelocityVector " << IP->CurrentVelocityVector->VecData[i]);
    reflexxes_IP_->CurrentAccelerationVector->VecData[i] = start_ddq[i];
    ////ROS_INFO_STREAM("CurrentAccelerationVector " << IP->CurrentAccelerationVector->VecData[i]);
    reflexxes_IP_->MaxVelocityVector->VecData[i] = v_max_allowed_[i];
    reflexxes_IP_->MaxAccelerationVector->VecData[i] = a_max_ltt_[i];
    reflexxes_IP_->MaxJerkVector->VecData[i] = j_max_ltt_[i];
    reflexxes_IP_->TargetPositionVector->VecData[i] = goal_q[i];
    reflexxes_IP_->TargetVelocityVector->VecData[i] = goal_dq[i];
    reflexxes_IP_->SelectionVector->VecData[i] = true;
  }

  std::vector<Motion> new_traj;
  std::vector<double> last_acc = start_ddq;
  double new_time = path_s_;
  new_traj.push_back(Motion(new_time, convertRMLVec(*reflexxes_IP_->CurrentPositionVector), convertRMLVec(*reflexxes_IP_->CurrentVelocityVector), convertRMLVec(*reflexxes_IP_->CurrentAccelerationVector)));
  /// Calculate trajectory
  while (ResultValue != ReflexxesAPI::RML_FINAL_STATE_REACHED) {
    // Calling the Reflexxes OTG algorithm
    ResultValue	=	reflexxes_RML_->RMLPosition(	*reflexxes_IP_, reflexxes_OP_, reflexxes_flags_);
                                        
    if (ResultValue < 0) {
        spdlog::error("An error occurred during safety_shield::calculateLongTermTrajectory {}", ResultValue);
        spdlog::error(reflexxes_OP_->GetErrorString());
        break;
    }
    *reflexxes_IP_->CurrentPositionVector = *reflexxes_OP_->NewPositionVector;
    *reflexxes_IP_->CurrentVelocityVector = *reflexxes_OP_->NewVelocityVector;
    *reflexxes_IP_->CurrentAccelerationVector = *reflexxes_OP_->NewAccelerationVector;
    new_time += sample_time_;
    std::vector<double> new_pos = convertRMLVec(*reflexxes_IP_->CurrentPositionVector);
    std::vector<double> new_vel = convertRMLVec(*reflexxes_IP_->CurrentVelocityVector);
    std::vector<double> new_acc = convertRMLVec(*reflexxes_IP_->CurrentAccelerationVector);
    std::vector<double> new_jerk;
    new_jerk.reserve(nb_joints_);
    for (int i = 0; i<nb_joints_; i++) {
      new_jerk.push_back((new_acc[i]-last_acc[i])/sample_time_);
    }
    new_traj.push_back(Motion(new_time, new_pos, new_vel, new_acc, new_jerk));
  }
  return LongTermTraj(new_traj, path_s_discrete_, sliding_window_k_);
}

std::vector<double> SafetyShield::convertRMLVec(const RMLDoubleVector& rml_vec) {
  std::vector<double> std_vec;
  for (int i = 0; i < nb_joints_; i++) {
    std_vec.push_back(rml_vec[i]);
  }
  return std_vec;
}

} // namespace safety_shield
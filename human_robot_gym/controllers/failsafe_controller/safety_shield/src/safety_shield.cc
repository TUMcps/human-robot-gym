#include "safety_shield/safety_shield.h"


namespace safety_shield {

SafetyShield::SafetyShield(bool activate_shield,
    int nb_joints, 
    double sample_time, 
    double t_buff, 
    double max_s_stop, 
    const std::vector<double> &v_max_allowed, 
    const std::vector<double> &a_max_allowed, 
    const std::vector<double> &j_max_allowed, 
    const std::vector<double> &a_max_path, 
    const std::vector<double> &j_max_path, 
    const LongTermTraj &long_term_trajectory, 
    const ros::Publisher &motion_pub,
    RobotReach* robot_reach,
    HumanReach* human_reach,
    Verify* verify,
    ControlCommandTranslator* translator,
    safety_shield::RvizMarker* rviz = nullptr):
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
  motion_pub_(motion_pub),
  robot_reach_(robot_reach),
  human_reach_(human_reach),
  verify_(verify),
  translator_(translator),
  rviz_(rviz)
{
  sliding_window_k_ = (int) std::floor(max_s_stop_/sample_time_);
  std::vector<double> prev_dq;
  for(int i = 0; i < 6; i++) {
    prev_dq.push_back(0.0);
  }

  is_safe_ = !activate_shield_;
  //ROS_INFO("PotentialTrajectory init");
  computesPotentialTrajectory(is_safe_, prev_dq);
  next_motion_ = determineNextMotion(is_safe_);
}


SafetyShield::~SafetyShield() {
  gazebo::client::shutdown();
};


bool SafetyShield::planSafetyShield(double pos, double vel, double acc, double ve, double a_max, double j_max, Path &path) {
  if (a_max < 0 || fabs(acc) > a_max) {
    ROS_DEBUG("planSafetyShield acc values incorrect. a_max = %f, acc = %f", a_max, acc);
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
      ROS_DEBUG("planSafetyShield calculated time negative. t01 = %f, t12 = %f, t23 = %f", t01, t12, t23);
      return false;
    }
    std::array<double,6> new_phases = { t01, t01+t12, t01+t12+t23, j01, j12, j23 };
    path.setPhases(new_phases);
    return true;
  } catch (const std::exception &exc) {
    ROS_ERROR_STREAM("Exception in SafetyShield::planSafetyShield: " << exc.what());
    return false;
  }
}

void SafetyShield::calculateMaxAccJerk(const std::vector<double> &prev_speed, const std::vector<double>& a_max_part, const std::vector<double>& j_max_part, double& a_max_manoeuvre, double& j_max_manoeuvre) {
  double new_c, new_d;

  double denom = std::abs(prev_speed[0]) + std::abs(a_max_part[0]) * max_s_stop_ + 1E-9;
  double min_c = (a_max_allowed_[0] - std::abs(a_max_part[0])) / denom;
  double min_d = (j_max_allowed_[0] - 3*std::abs(a_max_part[0])*a_max_allowed_[0] - std::abs(j_max_part[0])) / denom;
  ROS_DEBUG("calculateMaxAccJerk new_c = %f, new_d = %f, a_max_part[%d] = %f, j_max_part[%d] = %f", min_c, min_d, 0, std::abs(a_max_part[0]), 0, std::abs(j_max_part[0]));
  for (int i = 1; i < a_max_allowed_.size(); i++) {
    denom = std::abs(prev_speed[i]) + std::abs(a_max_part[i]) * max_s_stop_;
    new_c = (a_max_allowed_[i] - std::abs(a_max_part[i])) / denom;
    min_c = (new_c < min_c) ? new_c : min_c;
  
    new_d = (j_max_allowed_[i] - 3*std::abs(a_max_part[i])*new_c - std::abs(j_max_part[i])) / denom;
    min_d = (new_d < min_d) ? new_d : min_d;
    ROS_DEBUG("calculateMaxAccJerk new_c = %f, new_d = %f, a_max_part[%d] = %f, j_max_part[%d] = %f", new_c, new_d, i, std::abs(a_max_part[i]), i, std::abs(j_max_part[i]));
  }

  a_max_manoeuvre = (min_c < 0) ? 0 : min_c;
  j_max_manoeuvre = (min_d < 0) ? 0 : min_d;
  
  ROS_DEBUG("calculateMaxAccJerk denom = %f, a_max_manoeuvre = %f, j_max_manoeuvre = %f", denom, a_max_manoeuvre, j_max_manoeuvre);
}

custom_robot_msgs::StartGoalMotion SafetyShield::computesPotentialTrajectory(bool v, const std::vector<double> &prev_speed)
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
    
    //ROS_INFO("Current acceleration = %f, max acceleration = %f", failsafe_path_.getAcceleration(), a_max_manoeuvre);
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
    ROS_DEBUG_STREAM("Got start motion s_d = " << s_d << ", ds_d = " << ds_d << " dds_d = " << dds_d);
    // Always interpolate from current long term buffer
    custom_robot_msgs::Motion start_motion = interpolateFromTrajectory(s_d, ds_d, dds_d, long_term_trajectory_);
    start_motion.s = s_d;
    //ROS_DEBUG("Got start motion");
    // Calculate goal
    potential_path_.getFinalMotion(s_d, ds_d, dds_d);
    ROS_DEBUG_STREAM("Got final motion s_d = " << s_d << ", ds_d = " << ds_d << " dds_d = " << dds_d);
    custom_robot_msgs::Motion goal_motion;
    //ROS_DEBUG("Interpolate goal motion");
    if (new_ltt_) {
      goal_motion = interpolateFromTrajectory(s_d, ds_d, dds_d, new_long_term_trajectory_);
    } else {
      goal_motion = interpolateFromTrajectory(s_d, ds_d, dds_d, long_term_trajectory_);
    } 
    goal_motion.s = s_d;
    ROS_DEBUG("Got goal motion");
    custom_robot_msgs::StartGoalMotion s_g_motion;
    s_g_motion.header.frame_id = "map";
    s_g_motion.header.stamp = cycle_begin_time_;
    s_g_motion.start_motion = start_motion;
    s_g_motion.goal_motion = goal_motion;
    s_g_motion.duration = potential_path_.getPhase(3);
    return s_g_motion;
    //ROS_DEBUG("Published start goal motion");
  } catch (const std::exception &exc) {
    ROS_ERROR_STREAM("Exception in SafetyShield::computesPotentialTrajectory: " << exc.what());
    throw exc;
  }
}


custom_robot_msgs::Motion SafetyShield::determineNextMotion(bool is_safe) {
  custom_robot_msgs::Motion next_motion;
  double s_d, ds_d, dds_d;
  if (is_safe) {
    //ROS_INFO_STREAM("Determine next motion is_safe = true");
    // Fill potential buffer with position and velocity from recovery path
    if (recovery_path_.getPosition() >= failsafe_path_.getPosition()) {
      //ROS_INFO_STREAM("Determine next motion recovery_path.getPosition() >= failsafe_path.getPosition() = true");
      s_d = recovery_path_.getPosition();
      ds_d = recovery_path_.getVelocity();
      dds_d = recovery_path_.getAcceleration();
    }
    else {
      //ROS_INFO_STREAM("Determine next motion recovery_path.getPosition() >= failsafe_path.getPosition() = false");
      potential_path_.increment(sample_time_);
      s_d = potential_path_.getPosition();
      ds_d = potential_path_.getVelocity();
      dds_d = potential_path_.getAcceleration();
    }

    // Interpolate from new long term buffer
    if (new_ltt_) {
      //ROS_INFO_STREAM("Determine next motion new_ltt = true");
      next_motion = interpolateFromTrajectory(s_d, ds_d, dds_d, new_long_term_trajectory_);
    } else {
      //ROS_INFO_STREAM("Determine next motion new_ltt = false");
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

void SafetyShield::publishMotion(const custom_robot_msgs::Motion& motion) {
  // The motion command to publish
  modrob_workstation::RobotConfigCommanded command_msg;
  // Fill joint information
  for (int i = 0; i < motion.q.size(); i++) {
    modrob_workstation::JointConfigCommanded joint_msg = modrob_workstation::JointConfigCommanded();
    joint_msg.joint_angle = motion.q[i];
    joint_msg.joint_velocity = motion.dq[i];
    joint_msg.joint_acceleration = motion.ddq[i];
    joint_msg.joint_torque = 0.0;
    command_msg.joint_moves.push_back(joint_msg);
  }
  translator_->convertMsg(&command_msg);
}

custom_robot_msgs::Motion SafetyShield::interpolateFromTrajectory(double s, double ds, double dds, const LongTermTraj& trajectory) const {
  try {
    ROS_DEBUG_STREAM("Interpolate from trajectory with s = " << s);
    // Example: s=2.465, sample_time = 0.004 --> ind = 616.25
    ROS_ASSERT(sample_time_!=0);
    double ind = s/sample_time_;
    //ROS_DEBUG_STREAM("Interpolate from trajectory 2, ind = " << ind);
    double intpart;
    // Example: intpart = 616.0, ind_mod = 0.25
    double ind_mod = modf(ind, &intpart);
    //ROS_DEBUG_STREAM("Interpolate from trajectory 3, ind_mod = " << ind_mod);
    
    // floor(s/sample_time) + 1 ==> lower index
    int ind1 = static_cast<int>(intpart);
    // ceil(s/sample_time) + 1 ==> upper index
    int ind2 = static_cast<int>(ceil(ind));
    ROS_DEBUG_STREAM("Interpolate from trajectory, ind_1 = " << ind1 << " ind_2 = " << ind2);

    std::vector<double> q1 = trajectory.getNextMotionAtIndex(ind1).getAngle();
    std::vector<double> q2 = trajectory.getNextMotionAtIndex(ind2).getAngle();
    //std::vector<double> v1 = trajectory.getNextMotionAtIndex(ind1).getVelocity();
    //std::vector<double> v2 = trajectory.getNextMotionAtIndex(ind2).getVelocity();
    //ROS_INFO("s = %f, ind = %f, intpart = %f, ind_mod = %f, q[0,%i]=%f q[0,%i]=%f", s, ind, intpart, ind_mod, ind1, q1[0], ind2, q2[0]);
    std::vector<double> q;
    std::vector<double> dq;
    std::vector<double> ddq;
    std::vector<double> dddq = trajectory.getNextMotionAtIndex(ind1).getJerk();

    ROS_ASSERT_MSG(q1.size() == nb_joints_, "The angle information in the LTT at q1 is incorrect!");
    ROS_ASSERT_MSG(q2.size() == nb_joints_, "The angle information in the LTT at q2 is incorrect!");
    //ROS_DEBUG_STREAM("Interpolate from trajectory 5");
    for (int i = 0 ; i < nb_joints_; i++) {
      // Linearly interpolate between lower and upper index of position
      double q_clamped = q1[i] + ind_mod * (q2[i] - q1[i]);
      if (q_clamped < -max_q || q_clamped > max_q) {
        ROS_WARN_STREAM("Interpolated q value is outside of -pi and pi. q1 = " << q1[i] << " q2 = " << q2[i] << " ind_mod = " << ind_mod);
        q_clamped = std::clamp(q_clamped, -max_q, max_q);
      }
      q.push_back(q_clamped);
      // Calculate LTT velocity
      double v_max_int = (q2[i] - q1[i])/sample_time_;
      double v_int = v_max_int * ds;
      if (std::abs(v_int) > v_max_allowed_[i]) {
        ROS_DEBUG_STREAM("Interpolated v value is outside of |" << v_max_allowed_[i] << "|. q1 = " << q1[i] << " q2 = " << q2[i] << " ind_mod = " << ind_mod << " ds = " << ds);
        v_int = std::clamp(v_int, -v_max_allowed_[i], v_max_allowed_[i]);
      }
      dq.push_back(v_int);
      // Calculate Acceleration
      double a_int = v_max_int*dds;
      if (std::abs(a_int) > a_max_allowed_[i]) {
        ROS_DEBUG_STREAM("Interpolated a value is outside of |" << a_max_allowed_[i] << "|. q1 = " << q1[i] << " q2 = " << q2[i] << " ind_mod = " << ind_mod << " ds = " << ds << " dds = " << dds);
        a_int = std::clamp(a_int, -a_max_allowed_[i], a_max_allowed_[i]);
      }
      ddq.push_back(a_int);
    }
    //ROS_DEBUG_STREAM("Interpolate from trajectory 6");
    custom_robot_msgs::Motion motion = custom_robot_msgs::Motion();
    motion.s = s;
    motion.q = q;
    motion.dq = dq;
    motion.ddq = ddq;
    motion.dddq = dddq;
    return motion;
  } catch (const std::exception &exc) {
    ROS_ERROR_STREAM("Exception in SafetyShield::interpolateFromTrajectory: " << exc.what());
    return custom_robot_msgs::Motion();
  }

}

void SafetyShield::step(const ros::Time& cycle_begin_time) {
  //std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
  cycle_begin_time_ = cycle_begin_time;
  ROS_DEBUG("SafetyShield::step, new_ltt = %d, new_ltt_processed = %d, is_safe = %d, new_goal = %d", new_ltt_, new_ltt_processed_, is_safe_, new_goal_);
  try {
    // If the new LTT was processed at least once and is labeled safe, replace old LTT with new one.
    if (new_ltt_ && new_ltt_processed_ && is_safe_) {
      long_term_trajectory_ = new_long_term_trajectory_;
      new_ltt_ = false;
      new_goal_ = false;
    }

    // Check if there is a new goal motion
    if (new_goal_) {
      // Get current motion
      custom_robot_msgs::Motion current_motion = getCurrentMotion();
      // Check if current motion has acceleration and jerk values that lie in the plannable ranges
      bool is_plannable = checkCurrentMotionForReplanning(current_motion);
      ROS_DEBUG("is_plannable = %d", is_plannable);
      if (is_plannable) {
        // Check if the starting position of the last replanning was very close to the current position
        bool last_close = true;
        if (new_ltt_ == true) {
          for (int i = 0; i < current_motion.q.size(); i++) {
            if (std::abs(current_motion.q[i] - last_replan_start_motion_.q[i]) > 0.01) {
              last_close = false;
              break;
            }
          }
        } else {
          last_close = false;
        }
        // Only replan if the current joint position is different from the last.
        if (!last_close) {
          new_long_term_trajectory_ = calculateLongTermTrajectory(current_motion.q, current_motion.dq, 
            current_motion.ddq, new_goal_motion_.getAngle(), new_goal_motion_.getVelocity());
          last_replan_start_motion_ = current_motion;
        }
        new_ltt_ = true;
        new_ltt_processed_ = false;
      } else {
        ROS_DEBUG("current ddq = [%f, %f, %f], current dddq = [%f, %f, %f]", current_motion.ddq[0], current_motion.ddq[1], current_motion.ddq[2], current_motion.dddq[0], current_motion.dddq[1], current_motion.dddq[2]);
        new_ltt_ = false;
      }
    }
    // If there is a new long term trajectory (LTT), always override is_safe with false.
    if (new_ltt_ && !new_ltt_processed_) {
      is_safe_ = false;
    }
    // Compute a new potential trajectory
    custom_robot_msgs::StartGoalMotion sg_motion = computesPotentialTrajectory(is_safe_, next_motion_.dq);
    //std::chrono::steady_clock::time_point calc_path = std::chrono::steady_clock::now();
    if (activate_shield_) {
      // Compute the robot reachable set for the potential trajectory
      custom_robot_msgs::StartGoalCapsuleArray* robot_capsules = robot_reach_->reach(&sg_motion);
      // Compute the human reachable set for the potential trajectory
      custom_robot_msgs::CapsuleArray human_reach_capsules_P, human_reach_capsules_V, human_reach_capsules_A;
      human_reach_->humanReachabilityAnalysis(cycle_begin_time_.toSec(), sg_motion.duration, 
          human_reach_capsules_P, human_reach_capsules_V, human_reach_capsules_A);
      if (rviz_ != nullptr) {
        // Visualize to rviz
        rviz_->advanced_robot_callback(robot_capsules);
        rviz_->human_reach_callback_p(&human_reach_capsules_P);
        rviz_->human_reach_callback_v(&human_reach_capsules_V);
        rviz_->human_reach_callback_a(&human_reach_capsules_A);
      }
      // Verify if the robot and human reachable sets are collision free
      is_safe_ = verify_->verify_human_reach(robot_capsules, &human_reach_capsules_P, &human_reach_capsules_V, &human_reach_capsules_A);
    } else {
      is_safe_ = true;
    }
    //std::chrono::steady_clock::time_point verify_path = std::chrono::steady_clock::now();
    // Select the next motion based on the verified safety
    next_motion_ = determineNextMotion(is_safe_);
    // Publish the motion
    publishMotion(next_motion_);
    //std::chrono::steady_clock::time_point publish_path = std::chrono::steady_clock::now();
    //ROS_WARN("Time needed for calulating the path in mu s: %li, verifiying: %li, publishing %li, total: %li", std::chrono::duration_cast<std::chrono::microseconds>(calc_path - start).count(), std::chrono::duration_cast<std::chrono::microseconds>(verify_path - calc_path).count(), std::chrono::duration_cast<std::chrono::microseconds>(publish_path - verify_path).count(), std::chrono::duration_cast<std::chrono::microseconds>(publish_path - start).count());
    new_ltt_processed_ = true;
  } catch (const std::exception &exc) {
    ROS_ERROR_STREAM("Exception in SafetyShield::getNextCycle: " << exc.what());
  }
}

custom_robot_msgs::Motion SafetyShield::getCurrentMotion() {
  custom_robot_msgs::Motion current_pos;
  if (!recovery_path_.isCurrent()) {
    current_pos = interpolateFromTrajectory(failsafe_path_.getPosition(), failsafe_path_.getVelocity(), failsafe_path_.getAcceleration(), long_term_trajectory_);
  } else {
    current_pos = interpolateFromTrajectory(recovery_path_.getPosition(), recovery_path_.getVelocity(), recovery_path_.getAcceleration(), long_term_trajectory_); 
  }
  return current_pos;
}

bool SafetyShield::checkCurrentMotionForReplanning(const custom_robot_msgs::Motion& current_motion) {
  for (int i = 0; i < nb_joints_; i++) {
    if (std::abs(current_motion.ddq[i]) > a_max_ltt_[i]) {
      return false;
    }
  }
  return true;
}

void SafetyShield::newLongTermTrajectory(const custom_robot_msgs::MotionConstPtr& goal_motion) {
  try { 
    std::vector<double> motion_q;
    motion_q.reserve(goal_motion->q.size()); 
    std::vector<double> motion_dq;
    motion_q.reserve(goal_motion->q.size()); 
    for (int i = 0; i < goal_motion->q.size(); i++) {
      // TODO: replace with config max min values
      motion_q.push_back(std::clamp(goal_motion->q[i], -max_q, max_q));
      motion_dq.push_back(std::clamp(goal_motion->dq[i], -v_max_allowed_[i], v_max_allowed_[i]));
    }
    new_goal_motion_ = Motion(cycle_begin_time_.toSec(), motion_q, motion_dq);
    new_goal_ = true;
    new_ltt_ = false;
    new_ltt_processed_ = false;
    last_replan_start_motion_ = custom_robot_msgs::Motion();
    for (int i = 0; i < motion_q.size(); i++) {
      // fill with high values to guarantee this is not close to current joint position
      last_replan_start_motion_.q.push_back(-1234567);
    }
  } catch (const std::exception &exc) {
    ROS_ERROR_STREAM("Exception in SafetyShield::newLongTermTrajectory: " << exc.what());
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
    ROS_ASSERT(start_dq[i] <= v_max_allowed_[i]);
    ROS_ASSERT(start_ddq[i] <= a_max_ltt_[i]);
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
        ROS_ERROR_STREAM("An error occurred " << ResultValue);
        ROS_ERROR_STREAM(reflexxes_OP_->GetErrorString());
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
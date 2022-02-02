#include "safety_shield/human_reach.h"

namespace safety_shield {

HumanReach::HumanReach(int n_joints_meas, std::map<std::string, human_reach::jointPair>& body_link_joints, 
                       std::map<std::string, double>& thickness, std::vector<double>& max_v, std::vector<double>& max_a,
                       std::vector<double>& shoulder_ids, std::vector<double>& elbow_ids, std::vector<std::string>& wrist_names,
                       double measurement_error_pos, double measurement_error_vel, double delay):
  body_link_joints_(body_link_joints),
  measurement_error_pos_(measurement_error_pos),
  measurement_error_vel_(measurement_error_vel),
  delay_(delay)
{
  human_p_ = Articulated_P(measurement_error_pos, measurement_error_vel, delay, body_link_joints, thickness, max_v, shoulder_ids, elbow_ids, wrist_names);
  human_v_ = Articulated_V(measurement_error_pos, measurement_error_vel, delay, body_link_joints, thickness, max_v);
  human_a_ = Articulated_A(measurement_error_pos, measurement_error_vel, delay, body_link_joints, thickness, max_v, max_a);
  
  for (int i = 0; i < n_joints_meas; i++) {
    joint_pos_.push_back(Point());
    joint_vel_.push_back(Point());
  }
}

void HumanReach::measurement(const custom_robot_msgs::PositionsHeaderedConstPtr& human_joint_pos) {
  //ROS_DEBUG_STREAM("HumanReach::measurement");
  try {
    for (int i = 0; i < human_joint_pos->data.size(); i++) {
      Point new_point = Point(human_joint_pos->data[i].x, human_joint_pos->data[i].y, human_joint_pos->data[i].z);
      // If more than 1 measurement, calculate velocity
      if (last_meas_timestep_ != -1) {
        Point diff = Point::diff(new_point, joint_pos_[i]);
        double dt = human_joint_pos->header.stamp.toSec() - last_meas_timestep_;
        joint_vel_[i] = Point(diff.x/dt, diff.y/dt, diff.z/dt);
        has_second_meas_ = true;
      } 
      joint_pos_[i] = new_point;
    }
  last_meas_timestep_ = human_joint_pos->header.stamp.toSec();
  //ROS_INFO_STREAM("Human Mocap measurement received. Timestamp of meas was " << last_meas_timestep);
  } catch (const std::exception &exc) {
      ROS_ERROR_STREAM("Exception in HumanReach::measurement: " << exc.what());
  }
}


void HumanReach::humanReachabilityAnalysis(double t_command, double t_brake, 
                                          custom_robot_msgs::CapsuleArray& capsules_p, 
                                          custom_robot_msgs::CapsuleArray& capsules_v, 
                                          custom_robot_msgs::CapsuleArray& capsules_a) {
  ROS_DEBUG_STREAM("HumanReach::humanReachabilityAnalysis");
  try {
    // Time between reach command msg and last measurement plus the t_brake time.
    double t_reach = t_command-last_meas_timestep_ + t_brake;
    // Calculate reachable set
    std::vector<Capsule> reach_p = human_p_.update(joint_pos_, t_reach);
    capsules_p.header.stamp = ros::Time(t_command);
    for (auto& cap : reach_p) {
        capsules_p.capsules.push_back(cap.toCapsuleMsg());
    }
    std::vector<Capsule> reach_v = human_v_.update(joint_pos_, t_reach, joint_vel_);
    capsules_v.header.stamp = ros::Time(t_command);
    for (auto& cap : reach_v) {
        capsules_v.capsules.push_back(cap.toCapsuleMsg());
    }
    std::vector<Capsule> reach_a = human_a_.update(joint_pos_, t_reach, joint_vel_);
    capsules_a.header.stamp = ros::Time(t_command);
    for (auto& cap : reach_a) {
        capsules_a.capsules.push_back(cap.toCapsuleMsg());
    }
  } catch (const std::exception &exc) {
      ROS_ERROR_STREAM("Exception in HumanReach::humanReachabilityAnalysis: " << exc.what());
  }
}

} // namespace safety_shield



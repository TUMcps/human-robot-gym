#include "safety_shield/robot_reach.h"

namespace safety_shield {

RobotReach::RobotReach(std::vector<double> transformation_matrices, int nb_joints, std::vector<double> geom_par, 
    double x = 0, double y = 0, double z = 0, double roll = 0, double pitch = 0, double yaw = 0):
  nb_joints_(nb_joints)
{
  Eigen::Matrix4d transformation_matrix;
  double cr = cos(roll); double sr = sin(roll);
  double cp = cos(pitch); double sp = sin(pitch);
  double cy = cos(yaw); double sy = sin(yaw);
  transformation_matrix << cr*cp, cr*sp*sy-sr*cy, cr*sp*cy+sr*sy, x,
            sr*cp, sr*sp*sy+cr*cy, sr*sp*cy-cr*sy, y,
            -sp, cp*sy, cp*cy, z,
            0, 0, 0, 1;
  transformation_matrices_.push_back(transformation_matrix);
  for (int joint = 0; joint < nb_joints; joint++) {
    // Fill transformation matrix
    Eigen::Matrix4d transformation_matrix;
    for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 4; j++) {
        transformation_matrix(i, j) = transformation_matrices[16*joint + 4*i + j];
      }
    }
    transformation_matrices_.push_back(transformation_matrix);
    // Fill capsules
    
    Eigen::Vector4d p1;
    Eigen::Vector4d p2;
    for (int i = 0; i < 3; i++) {
      p1(i) = geom_par[7*joint + i];
      p2(i) = geom_par[7*joint + 3 + i];
    }
    reach_lib::Capsule capsule(vectorToPoint(p1), vectorToPoint(p2),geom_par[7*joint + 6]);
    robot_capsules_.push_back(capsule);
  }
  spdlog::info("Robot reach parameters created.");
}


reach_lib::Capsule RobotReach::transformCapsule(const int& n_joint, const Eigen::Matrix4d &T) {
  Eigen::Vector4d p1 = T * pointToVector(robot_capsules_[n_joint].p1_);
  Eigen::Vector4d p2 = T * pointToVector(robot_capsules_[n_joint].p2_);
  reach_lib::Capsule c(
    vectorToPoint(p1),
    vectorToPoint(p2),
    robot_capsules_[n_joint].r_);
  return c;
}

std::vector<reach_lib::Capsule> RobotReach::reach(Motion& start_config, Motion& goal_config, 
  double s_diff, std::vector<double> alpha_i) {
  try{
    Eigen::Matrix4d T_before = transformation_matrices_[0];
    Eigen::Matrix4d T_after = transformation_matrices_[0];
    std::vector<reach_lib::Capsule> reach_capsules;
    std::vector<double> q1 = start_config.getAngle();
    std::vector<double> q2 = goal_config.getAngle();
    for (int i = 0; i < nb_joints_; i++) { 
        // build capsule before
        forwardKinematic(q1[i], i, T_before);
        reach_lib::Capsule before = transformCapsule(i, T_before);
        // build capsule after
        forwardKinematic(q2[i], i, T_after);
        reach_lib::Capsule after = transformCapsule(i, T_after);

        // Caculate center of ball enclosing point p1 before and after
        reach_lib::Point p_1_k = (before.p1_ + after.p1_) * 0.5;
        // Caculate center of ball enclosing point p2 before and after
        reach_lib::Point p_2_k = (before.p2_ + after.p2_) * 0.5;
        // Calculate radius of ball enclosing point p1 before and after
        double r_1 = reach_lib::Point::norm(before.p1_-after.p1_) / 2 + alpha_i[i] * s_diff*s_diff/8 + robot_capsules_[i].r_;
        // Calculate radius of ball enclosing point p2 before and after
        double r_2 = reach_lib::Point::norm(before.p2_-after.p2_) / 2 + alpha_i[i+1] * s_diff*s_diff/8 + robot_capsules_[i].r_;
        // Enclosure capsule radius is max of ball around p1 and ball around p2 
        reach_capsules.push_back(reach_lib::Capsule(p_1_k, p_2_k, std::max(r_1, r_2)));
    }
    return reach_capsules;
  } catch (const std::exception &exc) {
    spdlog::error("Exception in RobotReach::reach: {}", exc.what());
    return {};
  }
}

} // namespace safety_shield
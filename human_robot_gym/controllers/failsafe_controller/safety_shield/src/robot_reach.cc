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
    Capsule capsule;
    for (int i = 0; i < 3; i++) {
      capsule.p1(i) = geom_par[7*joint + i];
      capsule.p2(i) = geom_par[7*joint + 3 + i];
    }
    capsule.p1(3) = 1;
    capsule.p2(3) = 1;
    capsule.r = geom_par[7*joint + 6];
    robot_capsules_.push_back(capsule);
  }
  ROS_INFO("Parameters created.");
}


custom_robot_msgs::Capsule RobotReach::transformCapsule(const int& n_joint, const Eigen::Matrix4d &T) {
  Eigen::Vector4d p1 = T * robot_capsules_[n_joint].p1;
  Eigen::Vector4d p2 = T * robot_capsules_[n_joint].p2;
  custom_robot_msgs::Capsule c;
  c.segment.p.x = p1(0);
  c.segment.p.y = p1(1);
  c.segment.p.z = p1(2);
  c.segment.q.x = p2(0);
  c.segment.q.y = p2(1);
  c.segment.q.z = p2(2);
  c.radius = robot_capsules_[n_joint].r;
  return c;
}


void RobotReach::computeCapsulesFromStartGoalSegments(const std::vector<custom_robot_msgs::Segment>& start_segments, 
                                                 const std::vector<custom_robot_msgs::Segment>& goal_segments, 
                                                 double s_diff, 
                                                 std::vector<custom_robot_msgs::Capsule>& output_capsules) {
  double epsilon = s_diff*s_diff/8;

  for (int i = 0; i < nb_joints_; i++) { 
    custom_robot_msgs::Capsule reachable_set;
    reachable_set.segment.p = geometry_helpers::pointAdd(start_segments[i].p, goal_segments[i].p, 2);
    reachable_set.segment.q = geometry_helpers::pointAdd(start_segments[i].q, goal_segments[i].q, 2);
    reachable_set.radius = std::max(geometry_helpers::norm(geometry_helpers::fromSegmentToVector(reachable_set.segment.p, start_segments[i].p)), 
                                    geometry_helpers::norm(geometry_helpers::fromSegmentToVector(reachable_set.segment.q, start_segments[i].q)))
                            + epsilon + robot_capsules_[i].r;
    output_capsules.push_back(reachable_set);
  }
}


void RobotReach::calculateStartGoalSegements(const std::vector<double> &q1, const std::vector<double> &q2, 
                                        std::vector<custom_robot_msgs::Segment>& start_segments, 
                                        std::vector<custom_robot_msgs::Segment>& goal_segments) {
  Eigen::Matrix4d T_before = transformation_matrices_[0];
  Eigen::Matrix4d T_after = transformation_matrices_[0];

  for (int i = 0; i < nb_joints_; i++) { 
    // q before
    forwardKinematic(q1[i], i, T_before);
    // q after
    forwardKinematic(q2[i], i, T_after);
    // Capsule before
    custom_robot_msgs::Capsule c_before = transformCapsule(i, T_before);
    start_segments.push_back(c_before.segment);
    // Capsule after
    custom_robot_msgs::Capsule c_after = transformCapsule(i, T_after);
    goal_segments.push_back(c_after.segment);
  }
}


custom_robot_msgs::StartGoalCapsuleArray* RobotReach::reach(const custom_robot_msgs::StartGoalMotion* potential_traj) {
  ROS_DEBUG_STREAM("RobotReach::reach");
  try{
    std::vector<custom_robot_msgs::Segment> start_segments;
    std::vector<custom_robot_msgs::Segment> goal_segments;
    calculateStartGoalSegements(potential_traj->start_motion.q, potential_traj->goal_motion.q, start_segments, goal_segments);
    double s_diff = potential_traj->goal_motion.s - potential_traj->start_motion.s;
    custom_robot_msgs::StartGoalCapsuleArray* robot_ra(new custom_robot_msgs::StartGoalCapsuleArray());
    robot_ra->starts = start_segments;
    robot_ra->goals = goal_segments;
    computeCapsulesFromStartGoalSegments(start_segments, goal_segments, s_diff, robot_ra->capsules);
    robot_ra->header.frame_id = potential_traj->header.frame_id;
    robot_ra->header.stamp = potential_traj->header.stamp;
    return robot_ra;
  } catch (const std::exception &exc) {
    ROS_ERROR_STREAM("Exception in RobotReach::reach: " << exc.what());
    return nullptr;
  }
}

} // namespace safety_shield
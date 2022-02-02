// -*- lsst-c++ -*-
/**
 * @file robot_reach.h
 * @brief Define the class for robot reachability analysis
 * @version 0.1
 * @copyright MIT License. Please see package.xml for further detail.
 */

#include <vector>
#include <algorithm>
#include <exception>

#include <Eigen/Dense>
#include <ros/ros.h>

#include "custom_robot_msgs/StartGoalCapsuleArray.h"
#include "custom_robot_msgs/CapsuleArray.h"
#include "custom_robot_msgs/StartGoalMotion.h"
#include "global_library/global_library.h"

#ifndef ROBOT_REACH_H
#define ROBOT_REACH_H

namespace safety_shield {

/**
 * @brief Class that calculates the robot reachable sets.
 */
class RobotReach {
 private:
  /**
   * @brief Simple data structure for capsules.
   * A capsule is defined by two points (forming a line) and a radius. 
   */ 
  struct Capsule {
    Eigen::Vector4d p1;
    Eigen::Vector4d p2;
    double r;
  };

  /**
   * @brief the number of joints of the robot
   */
  int nb_joints_;

  /**
   * @brief List of transforamtion matrices from joint to joint (fixed description, not including joint movements)
   */
  std::vector<Eigen::Matrix4d> transformation_matrices_;

  /**
   * @brief The enclosing capsules 
   */
  std::vector<Capsule> robot_capsules_;

public:

  /**
   * @brief A robot empty constructor
   */
  RobotReach() {}
  
  /**
   * @brief A robot basic constructor 
   *
   * @param transformation_matrices the transformation matrices
   * @param nb_joints the number of joints of the robot
   * @param geom_param the robot occupancy matrix
   * @param x initial x position of base
   * @param y initial y position of base
   * @param z initial z position of base
   * @param roll initial roll of base
   * @param pitch initial pitch of base
   * @param yaw initial yaw of base
   */
  RobotReach(std::vector<double> transformation_matrices, 
      int nb_joints, 
      std::vector<double> geom_par, 
      double x, double y, double z, 
      double roll, double pitch, double yaw);

  /**
   *  @brief A robot destructor
   */
  ~RobotReach() {}
    
  /**
   * @brief Computes the global transformation matrix of a given joint.
   *
   * @param q The joint angle
   * @param n_joint The number of joint 
   * @param T The current transformation matrix (Start with Identity). T will be modified in this function.
   */
  inline void forwardKinematic(const double &q, const int& n_joint, Eigen::Matrix4d &T) {
    // Transform T to new joint coordinate system
    T = T * transformation_matrices_[n_joint+1];
    Eigen::Matrix4d Rz;
    Rz << cos(q), -sin(q), 0, 0,
          sin(q), cos(q) , 0, 0,
          0     , 0      , 1, 0,
          0     , 0      , 0, 1;
    T = T * Rz;  
  }

  /**
   * @brief Transform the capsule of joint n by the transformation matrix T.
   * @return a custom robot msg capsule
   */
  custom_robot_msgs::Capsule transformCapsule(const int& n_joint, const Eigen::Matrix4d &T);

  /**
   * @brief calculate the start and goal segments of the robot given the start and goal joint angles 
   * @param[in] q1 The joint angles at the start of the motion
   * @param[in] q2 The joint angles at the end of the motion
   * @param[in] s_diff  the total duration of the trajectory
   * @param[out] start_segments The start segments vector to fill
   * @param[out] goal_segments The goal segments vector to fill
   */  
  void calculateStartGoalSegements(const std::vector<double> &q1, const std::vector<double> &q2,
                                   std::vector<custom_robot_msgs::Segment>& start_segments, 
                                   std::vector<custom_robot_msgs::Segment>& goal_segments);

  /**
   * @brief calculate reachable capsules from start and goal segments
   * @param[in] start_segments The segements at the begin of the motion
   * @param[in] goal_segments The segments at the end of the motion
   * @param[in] s_diff The traveled path distance
   * @param[out] output_capsules The casule array to fill
   */
  void computeCapsulesFromStartGoalSegments(const std::vector<custom_robot_msgs::Segment>& start_segments, 
                                            const std::vector<custom_robot_msgs::Segment>& goal_segments, 
                                            double s_diff, 
                                            std::vector<custom_robot_msgs::Capsule>& output_capsules);

  /**
   * @brief Calculates the reachable set from the new desired start and goal joint position.
   * 
   * Computes the reachable occupancy capsules of the robot to the ROS topic.
   * 
   * @returns Array of start goal capsules
   */
  custom_robot_msgs::StartGoalCapsuleArray* reach(const custom_robot_msgs::StartGoalMotion* potential_traj);
};
} // namespace safety_shield 

#endif // ROBOT_REACH_H

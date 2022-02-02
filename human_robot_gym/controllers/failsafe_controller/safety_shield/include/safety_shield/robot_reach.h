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
#include "spdlog/spdlog.h" 

#include <reach_lib.hpp>

#include "safety_shield/motion.h"

#ifndef ROBOT_REACH_H
#define ROBOT_REACH_H

namespace safety_shield {

/**
 * @brief Class that calculates the robot reachable sets.
 */
class RobotReach {
 private:
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
  std::vector<reach_lib::Capsule> robot_capsules_;

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

  Eigen::Vector4d pointToVector(const reach_lib::Point& p) {
      Eigen::Vector4d vec;
      vec << p.x, p.y, p.z, 1.0;
      return vec;
  }

  reach_lib::Point vectorToPoint(const Eigen::Vector4d& vec) {
      return reach_lib::Point(vec(0), vec(1), vec(2));
  }

  /**
   * @brief Transform the capsule of joint n by the transformation matrix T.
   * @return a custom robot msg capsule
   */
  reach_lib::Capsule transformCapsule(const int& n_joint, const Eigen::Matrix4d &T);

  /**
   * @brief Calculates the reachable set from the new desired start and goal joint position.
   * 
   * Computes the reachable occupancy capsules of the robot.
   * For a detailed proof of formality, please see: http://mediatum.ub.tum.de/doc/1443612/652879.pdf Chapter 3.4
   * 
   * @param[in] start_config The configuration of the robot in the beginning of the trajectory
   * @param[in] goal_config The configuration of the robot in the end of the trajectory
   * @param[in] s_diff The difference in the trajectory time parameter s for the given path
   * @param[in] alpha_i The maximum acceleration of each capsule point with resprect to the time parameter s for the given path
   * 
   * @returns Array of capsules
   */
  std::vector<reach_lib::Capsule> reach(Motion& start_config, Motion& goal_config,
    double s_diff, std::vector<double> alpha_i);
};
} // namespace safety_shield 

#endif // ROBOT_REACH_H

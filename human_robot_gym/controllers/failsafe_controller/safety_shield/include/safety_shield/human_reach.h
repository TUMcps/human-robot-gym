// -*- lsst-c++ -*/
/**
 * @file human_reach.h
 * @brief Defines the human reach class
 * @version 0.1
 * @copyright MIT License. Please see package.xml for further detail.
 */


#include <string>
#include <vector>
#include <map>
#include <exception>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>

#include "custom_robot_msgs/CapsuleArray.h"
#include "custom_robot_msgs/PositionsHeadered.h"
//#include "custom_robot_msgs/DoubleHeadered.h"
#include "reach_lib/Point.hpp"
#include "reach_lib/Capsule.hpp"
#include "reach_lib/Articulated_P.hpp"
#include "reach_lib/Articulated_V.hpp"
#include "reach_lib/Articulated_A.hpp"


#ifndef HUMAN_REACH_H
#define HUMAN_REACH_H

namespace safety_shield {
/**
 * @brief Each body part has a proximal and a distal joint (They can be the same.)
 */
typedef std::pair<int, int> jointPair;


/**
 * @brief Class handling the reachability analysis of the human.
 */
class HumanReach {
 private:

  /**
   * @brief Last measured timestep.
   */
  double last_meas_timestep_ = -1;

  /**
   * @brief Joint position measurements
   */
  std::vector<Point> joint_pos_;

  /**
   * @brief Calculated velocities
   */
  std::vector<Point> joint_vel_;

  /**
   * @brief Map the entries of body links to their proximal and distal joint.
   */
  std::map<std::string, human_reach::jointPair> body_link_joints_;

  /**
   * @brief The object for calculating the position based reachable set.
   */
  Articulated_P human_p_;

  /**
   * @brief The object for calculating the velocity based reachable set.
   */
  Articulated_V human_v_;

  /**
   * @brief The object for calculating the acceleration based reachable set.
   */
  Articulated_A human_a_;

  /**
   * @brief Maximal positional measurement error
   */
  double measurement_error_pos_ = 0.0;

  /**
   * @brief Maximal velocity measurement error
   */
  double measurement_error_vel_ = 0.0;

  /**
   * @brief Delay in measurement processing pipeline
   */
  double delay_ = 0.0;

  /**
   * @brief We need two measurements for velocity calculation.
   */
  bool has_second_meas_ = false;
  
public:
  /**
   * @brief Empty constructor
   */
  HumanReach() {}

  /**
   * @brief HumanReach constructor
   * @param[in] n_joints_meas Number of joints in the measurement
   * @param[in] measurement_error_pos Maximal positional measurement error
   * @param[in] measurement_error_vel Maximal velocity measurement error
   * @param[in] delay Delay in measurement processing pipeline
   * @param[in] joint_pair_map Maps the proximal and distal joint to a body part identified by a string (key: Name of body part, value: Proximal and distal joint index)
   * @param[in] thickness Defines the thickness of the body parts (key: Name of body part, value: Thickness of body part)
   * @param[in] max_v The maximum velocity of the joints
   * @param[in] max_a The maximum acceleration of the joints
   * @param[in] shoulder_ids The indices of the left and right shoulder joints 
   * @param[in] elbow_ids The indices of the left and right elbow joints 
   * @param[in] wrist_names The name identifiers of the two hands
  */
  HumanReach(int n_joints_meas, 
      std::map<std::string, human_reach::jointPair>& body_link_joints, 
      std::map<std::string, double>& thickness, 
      std::vector<double>& max_v, 
      std::vector<double>& max_a,
      std::vector<double>& shoulder_ids, 
      std::vector<double>& elbow_ids, 
      std::vector<std::string>& wrist_names,
      double measurement_error_pos, 
      double measurement_error_vel, 
      double delay);

  /**
   * @brief Destructor
   */
  ~HumanReach() {}

  /**
   * @brief Update the joint measurements.
   * @param[in] human_joint_pos The positions of the human joints and a header containing the simulation time.
   */
  void measurement(const custom_robot_msgs::PositionsHeaderedConstPtr& human_joint_pos);

  /**
   * @brief Calculate reachability analysis for given breaking time.
   * @param[in] t_command Current ROS time
   * @param[in] t_brake Breaking time message
   * @param[in] capsule_p reachable set for position approach
   * @param[in] capsule_v reachable set for velocity approach
   * @param[in] capsule_a reachable set for acceleration approach
   */
  void humanReachabilityAnalysis(double t_command, double t_brake,
      custom_robot_msgs::CapsuleArray& capsules_p, 
      custom_robot_msgs::CapsuleArray& capsules_v, 
      custom_robot_msgs::CapsuleArray& capsules_a);

};
} // namespace safety_shield
#endif // HUMAN_REACH_H

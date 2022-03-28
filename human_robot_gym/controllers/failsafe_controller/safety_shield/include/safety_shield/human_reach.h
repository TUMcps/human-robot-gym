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
#include <assert.h>

#include "spdlog/spdlog.h" 

#include "SaRA/reach_lib.hpp"


#ifndef HUMAN_REACH_H
#define HUMAN_REACH_H

namespace safety_shield {

/**
 * @brief Class handling the reachability analysis of the human.
 * 
 * This class holds all three articulated models (pos, vel, accel) of the human, 
 * and handels incoming measurements.
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
  std::vector<reach_lib::Point> joint_pos_;

  /**
   * @brief Calculated velocities
   */
  std::vector<reach_lib::Point> joint_vel_;

  /**
   * @brief Map the entries of body links to their proximal and distal joint.
   */
  std::map<std::string, reach_lib::jointPair> body_link_joints_;

  /**
   * @brief The object for calculating the position based reachable set.
   */
  reach_lib::ArticulatedPos human_p_;

  /**
   * @brief The object for calculating the velocity based reachable set.
   */
  reach_lib::ArticulatedVel human_v_;

  /**
   * @brief The object for calculating the acceleration based reachable set.
   */
  reach_lib::ArticulatedAccel human_a_;

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
   * @param[in] extremity_base_names The base joints of extremities, e.g., right / left shoulder, right / left hip socket
   * @param[in] extremity_end_names The end joints of extremities, e.g., right / left hand, right / left foot --> Is used for thickness of extremities
   * @param[in] extremity_length The max length of the extremities (related to extremity_base_names)
   * @param[in] wrist_names The name identifiers of the two hands
  */
  HumanReach(int n_joints_meas, 
      std::map<std::string, reach_lib::jointPair>& body_link_joints, 
      const std::map<std::string, double>& thickness, 
      std::vector<double>& max_v, 
      std::vector<double>& max_a,
      std::vector<std::string>& extremity_base_names, 
      std::vector<std::string>& extremity_end_names, 
      std::vector<double>& extremity_length, 
      double measurement_error_pos, 
      double measurement_error_vel, 
      double delay);

  /**
   * @brief Destructor
   */
  ~HumanReach() {}

  /**
   * @brief Update the joint measurements.
   * @param[in] human_joint_pos The positions of the human joints.
   * @param[in] time The simulation time.
   */
  void measurement(const std::vector<reach_lib::Point>& human_joint_pos, double time);

  /**
   * @brief Calculate reachability analysis for given breaking time.
   * 
   * Updates the values in human_p_, human_v_, human_a_.
   * Get the values afterwards with the getter functions!
   * 
   * @param[in] t_command Current time
   * @param[in] t_brake Time horizon of reachability analysis
   */
  void humanReachabilityAnalysis(double t_command, double t_brake);

  /**
   * @brief Get the Articulated Pos capsules
   * 
   * @return reach_lib::ArticulatedPos capsules
   */
  inline std::vector<reach_lib::Capsule> getArticulatedPosCapsules() {return reach_lib::get_capsules(human_p_);}

  /**
   * @brief Get the Articulated Vel capsules
   * 
   * @return reach_lib::ArticulatedVel capsules
   */
  inline std::vector<reach_lib::Capsule> getArticulatedVelCapsules() {return reach_lib::get_capsules(human_v_);}

  /**
   * @brief Get the Articulated Accel capsules
   * 
   * @return reach_lib::ArticulatedAccel capsules
   */
  inline std::vector<reach_lib::Capsule> getArticulatedAccelCapsules() {return reach_lib::get_capsules(human_a_);}

  /**
   * @brief Get the All Capsules of pos, vel, and accel
   * 
   * @return std::vector<std::vector<reach_lib::Capsule>> 
   */
  inline std::vector<std::vector<reach_lib::Capsule>> getAllCapsules() {
      return {getArticulatedPosCapsules(), getArticulatedVelCapsules(), getArticulatedAccelCapsules()};
  }
};
} // namespace safety_shield
#endif // HUMAN_REACH_H

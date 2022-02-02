// -*- lsst-c++ -*/
/**
 * @file safety_shield.h
 * @brief Defines the online verification class
 * @version 0.1
 * @copyright MIT License. Please see package.xml for further detail.
 */

#include <cmath>
#include <math.h>
#include <algorithm>
#include <vector>
#include <chrono>
#include <thread>

#include <ros/ros.h>

#ifndef SAFETY_SHIELD_H
#define SAFETY_SHIELD_H

namespace safety_shield {

/**
 * @brief Computes the failsafe trajectory
 */
class SafetyShield {
 private:
  /**
   * @brief sampling time
   */
  double sample_time_;

  /**
   * @brief Publishes that the calculation cycle is finished
   */
  ros::Publisher cycle_finished_pub_;

  /**
   * @brief the time when the loop begins
   */
  ros::Time cycle_begin_time_;
 
 public:
  /**
   * @brief A basic FailsafePathConsistent constructor
   */
  SafetyShield();

  /**
   * @brief A FailsafePathConsistent destructor
   */
  ~SafetyShield();

  /**
   * @brief Gets the information that the next simulation cycle (sample time) has started
   * @param cycle_begin_time ROS timestep of begin of current cycle
   */
  void step(const ros::Time& cycle_begin_time);

};
} // namespace safety_shield

#endif // safety_shield_H

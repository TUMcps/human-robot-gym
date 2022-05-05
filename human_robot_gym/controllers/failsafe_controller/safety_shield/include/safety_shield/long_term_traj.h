// -*- lsst-c++ -*/
/**
 * @file long_term_traj.h
 * @brief Defines the long term trajectory class
 * @version 0.1
 * @copyright MIT License. Please see package.xml for further detail.
 */

#include <algorithm>
#include <deque>
#include <utility>
#include <vector>

#include "spdlog/spdlog.h" // https://github.com/gabime/spdlog

#include "safety_shield/motion.h"

#ifndef LONG_TERM_TRAJ_H
#define LONG_TERM_TRAJ_H

namespace safety_shield {

/**
 * @brief A long term trajectory has a series of motions that the robot should move to.
 * 
 * When the trajectory is over, it will output its last position forever.
 */
class LongTermTraj {
 private:
  /**
   * @brief Trajectory length.
   */
  int length_;

  /**
   * @brief The long term trajectory.
   */
  std::vector<Motion> long_term_traj_;

  /**
   * @brief Maximum joint acceleration for next k steps.
   */
  std::vector<std::vector<double>> max_acceleration_window_;

  /**
   * @brief Maximum joint jerk for next k steps.
   */
  std::vector<std::vector<double>> max_jerk_window_;

  /**
   * @brief The current timestamp.
   */
  int current_pos_;

  /**
   * @brief Starting index if the trajectory is not initialized at s=0.
   */
  int starting_index_;

 public:
  /**
   * @brief Construct a new Long Term Traj object.
   */
  LongTermTraj():
    current_pos_(0),
    starting_index_(0),
    length_(1)
  {
      long_term_traj_.push_back(Motion(1));
  }

  /**
   * @brief Construct a new Long Term Traj object.
   * 
   * @param long_term_traj Vector of motions that make up the LTT.
   * @param sliding_window_k Size of sliding window for max acc and jerk calculationa
   */
  LongTermTraj(const std::vector<Motion> &long_term_traj, int starting_index=0, int sliding_window_k=10):
    long_term_traj_(long_term_traj),
    current_pos_(0),
    starting_index_(starting_index)
  {
    length_ = long_term_traj.size();
    calculate_max_acc_jerk_window(long_term_traj_, sliding_window_k);
  }

  /**
   * @brief Destroy the Long Term Traj object
   */
  ~LongTermTraj() {}

  inline void setLongTermTrajectory(const std::vector<Motion>& long_term_traj) {
    long_term_traj_ = long_term_traj;
    length_ = long_term_traj_.size();
  }

  /**
   * @brief Get the length of the LTT
   * 
   * @return int length
   */
  inline int getLength() const { return length_; }

  /**
   * @brief Get the current index of the LTT
   * 
   * @return int current_pos_
   */
  inline int getCurrentPos() const { return current_pos_; }

  /**
   * @brief Get the current motion of the LTT.
   * 
   * Equals getNextMotionAtIndex(0);
   * 
   * @return Current motion 
   */
  inline Motion getCurrentMotion() const { return long_term_traj_[current_pos_]; }

  /**
   * @brief Get the next motion of the LTT
   * 
   * Equals getNextMotionAtIndex(1);
   * 
   * @return Next motion 
   */
  inline Motion getNextMotion() const { return long_term_traj_[std::min(current_pos_+1, length_-1)]; }

  /**
   * @brief Get the motion at index from current pos
   * 
   * @param index steps from current pos
   * @return Motion 
   */
  inline Motion getNextMotionAtIndex(int index) const {
    return long_term_traj_[getTrajectoryIndex(index)];
  }

  inline int getTrajectoryIndex(int index) const {
    int desired_pos = std::min(index-starting_index_, length_-1);
    if (desired_pos < current_pos_) {
      //spdlog::debug("In LongTermTraj::getNextMotionAtIndex: desired_pos ({:08d}) < current_pos({:08d})", desired_pos, current_pos_);
      desired_pos = current_pos_;
    }
    return desired_pos;
  }

  /**
   * @brief Increment the current pos
   */
  inline void increasePosition() { current_pos_ = std::min(current_pos_+1, length_-1); }

  /**
   * @brief Get the Max Acceleration Window at index i
   * 
   * @param index 
   * @return std::vector<double> 
   */
  inline std::vector<double> getMaxAccelerationWindow(int index) const { 
    return max_acceleration_window_[getTrajectoryIndex(index)]; 
  }

  /**
   * @brief Get the Max Jerk Window at index i
   * 
   * @param index 
   * @return std::vector<double> 
   */
  inline std::vector<double> getMaxJerkWindow(int index) const { 
    return max_jerk_window_[getTrajectoryIndex(index)]; 
  }

  /**
   * @brief Calculate the maximum acceleration and jerk for each timestep with a sliding window approach.
   * 
   * Sets the `max_acceleration_window_` and `max_jerk_window_` values
   * 
   * @param long_term_traj Long term trajectory
   * @param k sliding window size
   */
  void calculate_max_acc_jerk_window(std::vector<Motion> &long_term_traj, int k);
};
}
#endif // LONG_TERM_TRAJ_H

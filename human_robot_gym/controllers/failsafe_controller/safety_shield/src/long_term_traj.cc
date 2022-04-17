#include "safety_shield/long_term_traj.h"

namespace safety_shield {

void LongTermTraj::calculate_max_acc_jerk_window(std::vector<Motion> &long_term_traj, int k) {
  int traj_length = long_term_traj.size();
  // It must be k <= trajectory length.
  k = std::min(traj_length, k);
  int n_joints = long_term_traj[0].getAngle().size();
  max_acceleration_window_.reserve(traj_length);
  max_jerk_window_.reserve(traj_length);

  // We use method 3 of https://www.geeksforgeeks.org/sliding-window-maximum-maximum-of-all-subarrays-of-size-k/
  std::vector<std::deque<int>> max_queue_acc;
  std::vector<std::deque<int>> max_queue_jerk;
  for (int j=0; j<n_joints; ++j) {
    max_queue_acc.push_back(std::deque<int>() );
    max_queue_jerk.push_back(std::deque<int>() );
  }
  /* Process first k (or first window)
    elements of array */
  int i;
  for (i = 0; i < k; ++i) {
    for (int j=0; j<n_joints; ++j) {
      // ACCELERATION
      // For every element, the previous smaller elements are useless so remove them from queue
      while (!max_queue_acc[j].empty() && long_term_traj[i].getAcceleration()[j] >= 
          long_term_traj[max_queue_acc[j].back()].getAcceleration()[j]) {
        // Remove from rear
        max_queue_acc[j].pop_back();
      }
      // Add new element at rear of queue
      max_queue_acc[j].push_back(i);

      // JERK
      while ((!max_queue_jerk[j].empty()) && long_term_traj[i].getJerk()[j] >= 
          long_term_traj[max_queue_jerk[j].back()].getJerk()[j]) {
        max_queue_jerk[j].pop_back();
      }
      max_queue_jerk[j].push_back(i);
    }
    
  }
  // Process rest of the elements,
  // i.e., from arr[k] to arr[n-1]
  for (; i < traj_length+k; ++i) {
    std::vector<double> max_acc; std::vector<double> max_jerk;
    max_acc.reserve(n_joints); max_jerk.reserve(n_joints);
    for (int j=0; j<n_joints; ++j) {
      // ACCELERATION
      // The element at the front of the queue is the largest element of previous window
      max_acc.push_back(long_term_traj[max_queue_acc[j].front()].getAcceleration()[j]);
      // Remove the elements which are out of this window
      while ((!max_queue_acc[j].empty()) && max_queue_acc[j].front() <= i - k) {
        // Remove from front of queue
        max_queue_acc[j].pop_front();
      }
      if (i<traj_length) {
        // Remove all elements smaller than the currently being added element (remove useless elements)
        while ((!max_queue_acc[j].empty()) && long_term_traj[i].getAcceleration()[j] >= long_term_traj[max_queue_acc[j].back()].getAcceleration()[j]) {
          max_queue_acc[j].pop_back();
        }
        // Add current element at the rear of Qi
        max_queue_acc[j].push_back(i);
      }

      // JERK
      max_jerk.push_back(long_term_traj[max_queue_jerk[j].front()].getJerk()[j]);
      while ((!max_queue_jerk[j].empty()) && max_queue_jerk[j].front() <= i - k) {
        max_queue_jerk[j].pop_front();
      }
      if (i<traj_length) {
        while ((!max_queue_jerk[j].empty()) && long_term_traj[i].getJerk()[j] >= long_term_traj[max_queue_jerk[j].back()].getJerk()[j]) {
          max_queue_jerk[j].pop_back();
        }
        max_queue_jerk[j].push_back(i);
      }
    }
    max_acceleration_window_.push_back(max_acc);
    max_jerk_window_.push_back(max_jerk);
  }
}


} // namespace safety_shield
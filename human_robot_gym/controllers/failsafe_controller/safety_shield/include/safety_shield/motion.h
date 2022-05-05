// -*- lsst-c++ -*/
/**
 * @file motion.h
 * @brief Defines the motion class
 * @version 0.1
 * @copyright MIT License. Please see package.xml for further detail.
 */

#include <vector>
#include <cmath>
#include <assert.h>

#ifndef MOTION_H
#define MOTION_H

namespace safety_shield {

/**
 * @brief Class that represents a motion of the differents modules of the robot
 */
class Motion {
 private:
  /**
   * @brief number of modules of the robot
   */
  int nb_modules_;

  /**
   * @brief Timestamp of the motion in seconds
   */
  double time_;

  /**
   * @brief Trajectory time
   * 
   */
  double s_;

  /**
   * @brief The joint angles
   */
  std::vector<double> q_;

  /**
   * @brief The joint velocities
   */
  std::vector<double> dq_;

  /**
   * @brief The joint accelerations
   */
  std::vector<double> ddq_;

  /**
   * @brief The joint jerks
   */
  std::vector<double> dddq_;
  
 public:
  /**
   * @brief Construct a new Motion object
   * 
   * Initializes the motion with no modules and t = 0
   */
  Motion():
    nb_modules_(0),
    time_(0) 
  {}

  /**
   * @brief Construct a new Motion object
   * A motion constructor taking a number of modules and returning the given motion with all parameters set to 0
   *
   * @param nb_modules the number of modules of the robot
   */
  Motion(int nb_modules);

  /**
   * @brief Construct a new Motion object
   * A motion constructor taking a situation (time and angle) and returning the given motion (with velocity and 
   * acceleration non initialized)
   *
   * @param time the time 
   * @param q the angle
   * @param s trajectory time parameter
  */
  Motion(double time, const std::vector<double> &q, double s=0.0);
  
  /**
   * @brief Construct a new Motion object
   * A motion constructor taking a situation (time and angle) and returning the given motion (with velocity and 
   * acceleration non initialized)
   *
   * @param time the time 
   * @param q the angle
   * @param dq velocity
   * @param s trajectory time parameter
  */
  Motion(double time, const std::vector<double> &q, const std::vector<double> &dq, double s=0.0);

  /**
   * @brief Construct a new Motion object
   *
   * @param time the time 
   * @param q angle
   * @param dq velocity
   * @param ddq acceleration 
   * @param s trajectory time parameter
  */
  Motion(double time, const std::vector<double> &q, const std::vector<double> &dq, 
        const std::vector<double> &ddq, double s=0.0);

  /**
   * @brief Construct a new Motion object
   *
   * @param time the time 
   * @param q angle
   * @param dq velocity
   * @param ddq acceleration 
   * @param dddq jerk
   * @param s trajectory time parameter
  */
  Motion(double time, const std::vector<double> &q, const std::vector<double> &dq, 
        const std::vector<double> &ddq, const std::vector<double> &dddq, double s=0.0);

  /**
   * @brief Destroy the Motion object
   */
  ~Motion() {}

  /**
   * @brief Check if this motion is at a complete stop (v, a, j) = 0 for all nb_modules_.
   * 
   * @param threshold Threshold for numerical impercision
   * @return true when motion is at a complete stop
   * @return false when motion is not at a complete stop
   */
  inline bool isStopped(double threshold=1e-4) {
    for (int i = 0; i < nb_modules_; i++) {
      if (std::abs(dq_[i]) > threshold ||
          std::abs(ddq_[i]) > threshold ||
          std::abs(dddq_[i]) > threshold) {
        return false;
      }
    }
    return true;
  }

  /**
   * @brief Checks if the given motion has the same position as this motion.
   * 
   * @param motion Motion to compare position to
   * @param threshold Threshold of similarity
   * @return true Position is the same
   * @return false Position is not the same
   */
  inline bool hasSamePos(Motion* motion, double threshold=1e-3) {
    assert(motion->getNbModules() == nb_modules_);
    // Check if traj starts at the same position
    for (int i = 0; i < nb_modules_; i++) {
      if (std::abs(q_[i]-motion->getAngle()[i]) > threshold) {
        return false;
      }
    }
    return true;
  }

  /**
   * @brief Checks if the given motion has the same velocity as this motion.
   * 
   * @param motion Motion to compare velocity to
   * @param threshold Threshold of similarity
   * @return true Velocity is the same
   * @return false Velocity is not the same
   */
  inline bool hasSameVel(Motion* motion, double threshold=1e-4) {
    assert(motion->getNbModules() == nb_modules_);
    // Check if traj starts at the same position
    for (int i = 0; i < nb_modules_; i++) {
      if (std::abs(dq_[i]-motion->getVelocity()[i]) > threshold) {
        return false;
      }
    }
    return true;
  }

  /**
   * @brief Checks if the given motion has the same acceleration as this motion.
   * 
   * @param motion Motion to compare acceleration to
   * @param threshold Threshold of similarity
   * @return true acceleration is the same
   * @return false acceleration is not the same
   */
  inline bool hasSameAcc(Motion* motion, double threshold=1e-4) {
    assert(motion->getNbModules() == nb_modules_);
    // Check if traj starts at the same position
    for (int i = 0; i < nb_modules_; i++) {
      if (std::abs(ddq_[i]-motion->getAcceleration()[i]) > threshold) {
        return false;
      }
    }
    return true;
  }

  /**
   * @brief Returns the number of modules
   * @return the number of modules
   */
  inline int getNbModules() { return nb_modules_; }

  /**
   * @brief Returns the time of the motion
   *
   * @return the time of the motion
   */
  inline double getTime() { return time_; }

  /**
   * @brief Return the trajectory time variable
   * 
   * @return double 
   */
  inline double getS() { return s_; }

  /**
   * @brief Returns the angle of the motion
   *
   * @return the angle of the motion
   */
  inline std::vector<double> getAngle() { return q_; }
  
  /**
   * @brief Returns the velocity of the motion
   *
   * @return the velocity of the motion
   */
  inline std::vector<double> getVelocity() { return dq_; }
  
  /**
   * @brief Returns the acceleration of the motion
   *
   * @return the acceleration of the motion
   */
  inline std::vector<double> getAcceleration() { return ddq_; }

  /**
   * @brief Returns the jerk of the motion
   *
   * @return the jerk of the motion
   */
  inline std::vector<double> getJerk() { return dddq_; }

  /**
   * @brief Sets the time of the motion
   *
   * @param new_time the new motion's time
   */
  inline void setTime(double new_time) { time_ = new_time; }

  /**
   * @brief Set the s value of the motion
   * 
   * @param new_s New s value
   */
  inline void setS(double new_s) { s_ = new_s; }

  /**
   * @brief Sets the angle
   *
   * @param new_q the new motion's angle
   */
  inline void setAngle(const std::vector<double> &new_q) { q_ = new_q; }
  
  /**
   * @brief Sets the velocity
   *
   * @param new_dq the new motion's velocity
   */
  inline void setVelocity(const std::vector<double> &new_dq) { dq_ = new_dq; }
  
  /**
   * @brief Sets the acceleration
   *
   * @param new_acceleration the new motion's acceleration
   */
  inline void setAcceleration(const std::vector<double> &new_ddq) { ddq_ = new_ddq; }

  /**
   * @brief Sets the jerk
   *
   * @param new_jerk the new motion's jerk
   */
  inline void setJerk(const std::vector<double> &new_dddq) { dddq_ = new_dddq; }
};
} // namespace safety_shield

#endif // MOTION_H
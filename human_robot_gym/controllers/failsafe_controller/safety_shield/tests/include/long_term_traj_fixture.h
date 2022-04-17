// -*- lsst-c++ -*/
/**
 * @file long_term_traj_fixture.h
 * @brief Defines the test fixture for verify long term trajectory class
 * @version 0.1
 * @copyright MIT License. Please see package.xml for further detail.
 */

#include <gtest/gtest.h>

#include "safety_shield/motion.h"
#include "safety_shield/long_term_traj.h"

#ifndef LONG_TERM_TRAJ_FIXTURE_H
#define LONG_TERM_TRAJ_FIXTURE_H

namespace safety_shield {

/**
 * @brief Test fixture for verify long term trajectory class
 */
class LongTermTrajTest : public ::testing::Test {
 protected:
  /**
   * @brief The LTT object
   */
  LongTermTraj long_term_trajectory_;

  /**
   * @brief Create the LTT object
   */
  void SetUp() override {
      long_term_trajectory_ = LongTermTraj();
  }
};
} // namespace safety_shield

#endif // LONG_TERM_TRAJ_FIXTURE_H
// -*- lsst-c++ -*/
/**
 * @file verify_iso_fixture.h
 * @brief Defines the test fixture for verify ISO class
 * @version 0.1
 * @copyright MIT License. Please see package.xml for further detail.
 */

#include <gtest/gtest.h>

#include "safety_shield/verify_iso.h"

#ifndef VERFIY_ISO_FIXTURE_H
#define VERFIY_ISO_FIXTURE_H

namespace safety_shield {

/**
 * @brief Test fixture for verify ISO class
 */
class VerifyIsoTest : public ::testing::Test {
 protected:
  /**
   * @brief The verify iso object
   */
  VerifyISO verify_iso_;

  /**
   * @brief Create the verify iso object
   */
  void SetUp() override {
      verify_iso_ = VerifyISO();
  }
};
} // namespace safety_shield

#endif // VERFIY_ISO_FIXTURE_H
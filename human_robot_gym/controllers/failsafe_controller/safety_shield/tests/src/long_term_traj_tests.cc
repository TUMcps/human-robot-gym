#include <vector>

#include <gtest/gtest.h>
#include <spdlog/spdlog.h>

#include "long_term_traj_fixture.h"
#include "safety_shield/long_term_traj.h"
#include "safety_shield/motion.h"

namespace safety_shield {

TEST_F(LongTermTrajTest, MaxAccWindowTest){
  std::vector<Motion> mo_vec;
  int n_joints = 2;
  std::vector<double> p0;
  std::vector<double> v0;
  std::vector<double> j0;
  for (int i = 0; i < n_joints; i++) {
    p0.push_back(0.0);
    v0.push_back(0.0);
    j0.push_back(0.0);
  }
  std::vector<double> a0;
  a0.push_back(12);
  a0.push_back(13);
  Motion m0(0, p0, v0, a0, j0);
  mo_vec.push_back(m0);
  std::vector<double> a1;
  a1.push_back(1);
  a1.push_back(2);
  Motion m1(1, p0, v0, a1, j0);
  mo_vec.push_back(m1);
  std::vector<double> a2;
  a2.push_back(78);
  a2.push_back(79);
  Motion m2(2, p0, v0, a2, j0);
  mo_vec.push_back(m2);
  std::vector<double> a3;
  a3.push_back(90);
  a3.push_back(91);
  Motion m3(3, p0, v0, a3, j0);
  mo_vec.push_back(m3);
  std::vector<double> a4;
  a4.push_back(57);
  a4.push_back(58);
  Motion m4(4, p0, v0, a4, j0);
  mo_vec.push_back(m4);
  std::vector<double> a5;
  a5.push_back(89);
  a5.push_back(90);
  Motion m5(5, p0, v0, a5, j0);
  mo_vec.push_back(m5);
  std::vector<double> a6;
  a6.push_back(56);
  a6.push_back(57);
  Motion m6(6, p0, v0, a6, j0);
  mo_vec.push_back(m6);
  long_term_trajectory_.setLongTermTrajectory(mo_vec);
  long_term_trajectory_.calculate_max_acc_jerk_window(mo_vec, 3);

  EXPECT_EQ(long_term_trajectory_.getMaxAccelerationWindow(0)[0], 78);
  EXPECT_EQ(long_term_trajectory_.getMaxAccelerationWindow(1)[0], 90);
  EXPECT_EQ(long_term_trajectory_.getMaxAccelerationWindow(2)[0], 90);
  EXPECT_EQ(long_term_trajectory_.getMaxAccelerationWindow(3)[0], 90);
  EXPECT_EQ(long_term_trajectory_.getMaxAccelerationWindow(4)[0], 89);
  EXPECT_EQ(long_term_trajectory_.getMaxAccelerationWindow(5)[0], 89);
  EXPECT_EQ(long_term_trajectory_.getMaxAccelerationWindow(6)[0], 56);
  EXPECT_EQ(long_term_trajectory_.getMaxAccelerationWindow(0)[1], 79);
  EXPECT_EQ(long_term_trajectory_.getMaxAccelerationWindow(1)[1], 91);
  EXPECT_EQ(long_term_trajectory_.getMaxAccelerationWindow(2)[1], 91);
  EXPECT_EQ(long_term_trajectory_.getMaxAccelerationWindow(3)[1], 91);
  EXPECT_EQ(long_term_trajectory_.getMaxAccelerationWindow(4)[1], 90);
  EXPECT_EQ(long_term_trajectory_.getMaxAccelerationWindow(5)[1], 90);
  EXPECT_EQ(long_term_trajectory_.getMaxAccelerationWindow(6)[1], 57);
}

TEST_F(LongTermTrajTest, MaxJerkWindowTest){
  std::vector<Motion> mo_vec;
  int n_joints = 2;
  std::vector<double> p0;
  std::vector<double> v0;
  std::vector<double> a0;
  for (int i = 0; i < n_joints; i++) {
    p0.push_back(0.0);
    v0.push_back(0.0);
    a0.push_back(0.0);
  }
  std::vector<double> j0;
  j0.push_back(12);
  j0.push_back(13);
  Motion m0(0, p0, v0, a0, j0);
  mo_vec.push_back(m0);
  std::vector<double> j1;
  j1.push_back(1);
  j1.push_back(2);
  Motion m1(1, p0, v0, a0, j1);
  mo_vec.push_back(m1);
  std::vector<double> j2;
  j2.push_back(78);
  j2.push_back(79);
  Motion m2(2, p0, v0, a0, j2);
  mo_vec.push_back(m2);
  std::vector<double> j3;
  j3.push_back(90);
  j3.push_back(91);
  Motion m3(3, p0, v0, a0, j3);
  mo_vec.push_back(m3);
  std::vector<double> j4;
  j4.push_back(57);
  j4.push_back(58);
  Motion m4(4, p0, v0, a0, j4);
  mo_vec.push_back(m4);
  std::vector<double> j5;
  j5.push_back(89);
  j5.push_back(90);
  Motion m5(5, p0, v0, a0, j5);
  mo_vec.push_back(m5);
  std::vector<double> j6;
  j6.push_back(56);
  j6.push_back(57);
  Motion m6(6, p0, v0, a0, j6);
  mo_vec.push_back(m6);
  long_term_trajectory_.setLongTermTrajectory(mo_vec);
  long_term_trajectory_.calculate_max_acc_jerk_window(mo_vec, 3);

  EXPECT_EQ(long_term_trajectory_.getMaxJerkWindow(0)[0], 78);
  EXPECT_EQ(long_term_trajectory_.getMaxJerkWindow(1)[0], 90);
  EXPECT_EQ(long_term_trajectory_.getMaxJerkWindow(2)[0], 90);
  EXPECT_EQ(long_term_trajectory_.getMaxJerkWindow(3)[0], 90);
  EXPECT_EQ(long_term_trajectory_.getMaxJerkWindow(4)[0], 89);
  EXPECT_EQ(long_term_trajectory_.getMaxJerkWindow(5)[0], 89);
  EXPECT_EQ(long_term_trajectory_.getMaxJerkWindow(6)[0], 56);
  EXPECT_EQ(long_term_trajectory_.getMaxJerkWindow(0)[1], 79);
  EXPECT_EQ(long_term_trajectory_.getMaxJerkWindow(1)[1], 91);
  EXPECT_EQ(long_term_trajectory_.getMaxJerkWindow(2)[1], 91);
  EXPECT_EQ(long_term_trajectory_.getMaxJerkWindow(3)[1], 91);
  EXPECT_EQ(long_term_trajectory_.getMaxJerkWindow(4)[1], 90);
  EXPECT_EQ(long_term_trajectory_.getMaxJerkWindow(5)[1], 90);
  EXPECT_EQ(long_term_trajectory_.getMaxJerkWindow(6)[1], 57);
}

} // namespace safety_shield

int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}